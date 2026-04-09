#include "message/AlgoPublisher.h"

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <mutex>

extern "C" {
#include <amqp.h>
#include <amqp_tcp_socket.h>
}

namespace {

static bool amqp_ok(amqp_rpc_reply_t r, const char* ctx) {
    switch (r.reply_type) {
        case AMQP_RESPONSE_NORMAL:
            return true;
        case AMQP_RESPONSE_NONE:
            std::cerr << "AlgoPublisher amqp response none: " << ctx << std::endl;
            break;
        case AMQP_RESPONSE_LIBRARY_EXCEPTION:
            std::cerr << "AlgoPublisher amqp lib exception: " << ctx
                      << " err=" << amqp_error_string2(r.library_error) << std::endl;
            break;
        case AMQP_RESPONSE_SERVER_EXCEPTION:
            if (r.reply.id == AMQP_CONNECTION_CLOSE_METHOD) {
                auto* m = (amqp_connection_close_t*)r.reply.decoded;
                std::cerr << "AlgoPublisher server connection close reply-code=" << m->reply_code
                          << " text=" << std::string((char*)m->reply_text.bytes, m->reply_text.len) << std::endl;
            } else if (r.reply.id == AMQP_CHANNEL_CLOSE_METHOD) {
                auto* m = (amqp_channel_close_t*)r.reply.decoded;
                std::cerr << "AlgoPublisher server channel close reply-code=" << m->reply_code
                          << " text=" << std::string((char*)m->reply_text.bytes, m->reply_text.len) << std::endl;
            } else {
                std::cerr << "AlgoPublisher server exception: " << ctx << " method id=" << r.reply.id << std::endl;
            }
            break;
    }
    return false;
}

static std::string routing_key_from_env(const char* env_name, const char* fallback) {
    const char* v = std::getenv(env_name);
    if (v && *v) return v;
    return fallback;
}

}  // namespace

struct AlgoPublisher::Impl {
    explicit Impl(const RabbitMQConfig& cfgIn)
        : cfg(cfgIn),
          conn(nullptr),
          connected(false),
          channel_open(false),
          message_ttl_ms(0) {
        const char* e = std::getenv("ALGO_PUBLISH_EXCHANGE");
        const char* q = std::getenv("ALGO_PUBLISH_QUEUE");
        const char* b = std::getenv("ALGO_PUBLISH_BINDING_KEY");
        const char* ttl = std::getenv("ALGO_PUBLISH_TTL_MS");
        exchange = e ? e : cfg.algoToDispExchange;
        queue = q ? q : cfg.algoToDispQueue;
        binding_key = b ? b : cfg.algoToDispBindingKey;
        if (binding_key.empty()) binding_key = "#";
        if (ttl && *ttl) {
            try {
                int tmp = std::stoi(ttl);
                if (tmp >= 0) message_ttl_ms = tmp;
            } catch (...) {}
        }
        expiration_str = std::to_string(std::max(0, message_ttl_ms));
    }

    ~Impl() {
        close();
    }

    bool ensure_connected_locked() {
        if (connected && conn) return true;
        close_locked();
        conn = amqp_new_connection();
        amqp_socket_t* sock = amqp_tcp_socket_new(conn);
        if (!sock) {
            std::cerr << "AlgoPublisher: cannot create TCP socket" << std::endl;
            close_locked();
            return false;
        }
        if (amqp_socket_open(sock, cfg.host.c_str(), cfg.port)) {
            std::cerr << "AlgoPublisher: socket open failed" << std::endl;
            close_locked();
            return false;
        }
        if (!amqp_ok(amqp_login(conn, cfg.vhost.c_str(), 0, 131072, 0,
                                AMQP_SASL_METHOD_PLAIN,
                                cfg.username.c_str(), cfg.password.c_str()),
                     "login")) {
            close_locked();
            return false;
        }
        amqp_channel_open(conn, 1);
        if (!amqp_ok(amqp_get_rpc_reply(conn), "channel.open")) {
            close_locked();
            return false;
        }
        channel_open = true;
        amqp_exchange_declare(conn, 1,
                              amqp_cstring_bytes(exchange.c_str()),
                              amqp_cstring_bytes("fanout"),
                              0, 1, 0, 0, amqp_empty_table);
        if (!amqp_ok(amqp_get_rpc_reply(conn), "exchange.declare")) {
            close();
            return false;
        }
        amqp_table_entry_t entries[1];
        amqp_table_t args{0, nullptr};
        if (message_ttl_ms > 0) {
            entries[0].key = amqp_cstring_bytes("x-message-ttl");
            entries[0].value.kind = AMQP_FIELD_KIND_I32;
            entries[0].value.value.i32 = message_ttl_ms;
            args.entries = entries;
            args.num_entries = 1;
        }
        amqp_queue_declare(conn, 1,
                           amqp_cstring_bytes(queue.c_str()),
                           0, 1, 0, 0, args);
        if (!amqp_ok(amqp_get_rpc_reply(conn), "queue.declare")) {
            close_locked();
            return false;
        }
        amqp_queue_bind(conn, 1,
                        amqp_cstring_bytes(queue.c_str()),
                        amqp_cstring_bytes(exchange.c_str()),
                        amqp_cstring_bytes(binding_key.c_str()),
                        amqp_empty_table);
        if (!amqp_ok(amqp_get_rpc_reply(conn), "queue.bind")) {
            close_locked();
            return false;
        }
        connected = true;
        return true;
    }

    bool publish(const std::string& routingKey, const std::string& payload) {
        std::lock_guard<std::mutex> lk(mu);
        if (!ensure_connected_locked()) return false;
        amqp_bytes_t bytes;
        bytes.len = payload.size();
        bytes.bytes = const_cast<char*>(payload.data());
        amqp_basic_properties_t props;
        std::memset(&props, 0, sizeof(props));
        props._flags = AMQP_BASIC_CONTENT_TYPE_FLAG | AMQP_BASIC_DELIVERY_MODE_FLAG;
        props.content_type = amqp_cstring_bytes("application/json");
        props.delivery_mode = 2;
        if (message_ttl_ms > 0) {
            props._flags |= AMQP_BASIC_EXPIRATION_FLAG;
            props.expiration = amqp_cstring_bytes(expiration_str.c_str());
        }
        int rc = amqp_basic_publish(conn, 1,
                                    amqp_cstring_bytes(exchange.c_str()),
                                    amqp_cstring_bytes(routingKey.c_str()),
                                    0, 0, &props, bytes);
        if (rc) {
            std::cerr << "AlgoPublisher: publish failed rc=" << rc << std::endl;
            close_locked();
            return false;
        }
        return true;
    }

    void close_locked() {
        if (conn) {
            if (channel_open) {
                amqp_channel_close(conn, 1, AMQP_REPLY_SUCCESS);
                channel_open = false;
            }
            amqp_connection_close(conn, AMQP_REPLY_SUCCESS);
            amqp_destroy_connection(conn);
        }
        conn = nullptr;
        connected = false;
    }

    void close() {
        std::lock_guard<std::mutex> lk(mu);
        close_locked();
    }

    RabbitMQConfig cfg;
    std::string exchange;
    std::string queue;
    std::string binding_key;
    int message_ttl_ms;
    std::string expiration_str;
    amqp_connection_state_t conn;
    bool connected;
    bool channel_open;
    std::mutex mu;
};

AlgoPublisher::AlgoPublisher(const RabbitMQConfig& cfg)
    : impl_(new Impl(cfg)) {}

AlgoPublisher::~AlgoPublisher() {
    delete impl_;
    impl_ = nullptr;
}

bool AlgoPublisher::publishJson(const std::string& routingKey, const nlohmann::json& body) {
    if (!impl_) return false;
    return impl_->publish(routingKey, body.dump());
}

bool AlgoPublisher::sendServiceInitialize(const std::string& serviceName) {
    nlohmann::json body = {
        {"serviceName", serviceName}
    };
    return publishJson(routing_key_from_env("ALGO_INIT_ROUTING_KEY", "AlgoServiceInitialize"), body);
}

bool AlgoPublisher::sendHeartbeat(const std::string& serviceName, bool allocating) {
    long long ts = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    nlohmann::json body = {
        {"serviceName", serviceName},
        {"timeStamp", ts},
        {"allocating", allocating ? 1 : 0}
    };
    return publishJson(routing_key_from_env("ALGO_HEARTBEAT_ROUTING_KEY", "AlgoServiceHeartbeat"), body);
}

bool AlgoPublisher::sendRunInfo(const std::string& deviceId,
                                const std::string& desc,
                                const std::string& timeStr) {
    nlohmann::json body = {
        {"id", deviceId},
        {"runDesc", desc},
        {"runTime", timeStr}
    };
    return publishJson(routing_key_from_env("ALGO_RUNINFO_ROUTING_KEY", "SendRunInfo"), body);
}

bool AlgoPublisher::sendAlertInfo(const std::string& deviceId,
                                  int alertCode,
                                  const std::string& alertLevel,
                                  const std::string& alertDesc,
                                  const std::string& timeStr) {
    nlohmann::json body = {
        {"id", deviceId},
        {"alertCode", alertCode},
        {"alertLevel", alertLevel},
        {"alertDesc", alertDesc},
        {"alertTime", timeStr}
    };
    return publishJson(routing_key_from_env("ALGO_ALERT_ROUTING_KEY", "SendAlertInfo"), body);
}

bool AlgoPublisher::sendPathResponse(const nlohmann::json& body) {
    return publishJson(routing_key_from_env("ALGO_PATH_RESPONSE_ROUTING_KEY", "RobotPathResponse"), body);
}

bool AlgoPublisher::sendAroundPathResponse(const nlohmann::json& body) {
    return publishJson(routing_key_from_env("ALGO_AROUND_PATH_RESPONSE_ROUTING_KEY", "RobotAroundPathResponse"), body);
}

bool AlgoPublisher::sendTrailResponse(const nlohmann::json& body) {
    return publishJson(routing_key_from_env("ALGO_TRAIL_RESPONSE_ROUTING_KEY", "RobotTrailResponse"), body);
}
