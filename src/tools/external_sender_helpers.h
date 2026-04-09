// 通用的 RabbitMQ 发送辅助函数，供 external_*_sender 使用
#ifndef EXTERNAL_SENDER_HELPERS_H
#define EXTERNAL_SENDER_HELPERS_H

extern "C" {
#include <amqp.h>
#include <amqp_tcp_socket.h>
}

#include <iostream>
#include <string>
#include <cstring>

#include "common/RabbitMQConfig.h"

namespace ExternalSender {

inline bool amqp_ok(amqp_rpc_reply_t r, const char* ctx) {
    if (r.reply_type != AMQP_RESPONSE_NORMAL) {
        std::cerr << "AMQP error in " << ctx << ": reply_type=" << r.reply_type << std::endl;
        return false;
    }
    return true;
}

struct PublishOptions {
    std::string exchange;
    std::string exchangeType = "fanout";
    std::string queue;
    std::string routingKey;
    std::string bindingKey = "#";
    int messageTtlMs = 0;  // 默认不设置 TTL，可由 EXT_MESSAGE_TTL_MS 覆盖
    int maxPriority = 0;   // 默认不设置优先级上限，可通过 EXT_QUEUE_MAX_PRIORITY 覆盖
};

inline bool PublishPayload(
    const RabbitMQConfig& cfg,
    PublishOptions opt,
    const std::string& payload
) {
    if (const char* ttlEnv = std::getenv("EXT_MESSAGE_TTL_MS")) {
        try { opt.messageTtlMs = std::max(0, std::stoi(ttlEnv)); } catch (...) {}
    }
    if (const char* priorityEnv = std::getenv("EXT_QUEUE_MAX_PRIORITY")) {
        try { opt.maxPriority = std::max(0, std::stoi(priorityEnv)); } catch (...) {}
    }
    amqp_connection_state_t conn = amqp_new_connection();
    amqp_socket_t* sock = amqp_tcp_socket_new(conn);
    if (!sock) {
        std::cerr << "ExternalSender: cannot create TCP socket" << std::endl;
        amqp_destroy_connection(conn);
        return false;
    }
    if (amqp_socket_open(sock, cfg.host.c_str(), cfg.port)) {
        std::cerr << "ExternalSender: socket open failed" << std::endl;
        amqp_destroy_connection(conn);
        return false;
    }
    if (!amqp_ok(amqp_login(conn, cfg.vhost.c_str(), 0, 131072, 0,
                            AMQP_SASL_METHOD_PLAIN,
                            cfg.username.c_str(), cfg.password.c_str()),
                 "login")) {
        amqp_destroy_connection(conn);
        return false;
    }
    amqp_channel_open(conn, 1);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "channel.open")) {
        amqp_connection_close(conn, AMQP_REPLY_SUCCESS);
        amqp_destroy_connection(conn);
        return false;
    }

    amqp_exchange_declare(conn, 1,
                          amqp_cstring_bytes(opt.exchange.c_str()),
                          amqp_cstring_bytes(opt.exchangeType.c_str()),
                          0, 1, 0, 0, amqp_empty_table);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "exchange.declare")) {
        amqp_connection_close(conn, AMQP_REPLY_SUCCESS);
        amqp_destroy_connection(conn);
        return false;
    }

    amqp_table_entry_t entries[2];
    amqp_table_t args{0, nullptr};
    int entryIdx = 0;
    if (opt.maxPriority > 0) {
        entries[entryIdx].key = amqp_cstring_bytes("x-max-priority");
        entries[entryIdx].value.kind = AMQP_FIELD_KIND_U8;
        entries[entryIdx].value.value.u8 = static_cast<uint8_t>(std::min(opt.maxPriority, 255));
        entryIdx++;
    }
    if (opt.messageTtlMs > 0) {
        entries[entryIdx].key = amqp_cstring_bytes("x-message-ttl");
        entries[entryIdx].value.kind = AMQP_FIELD_KIND_I32;
        entries[entryIdx].value.value.i32 = opt.messageTtlMs;
        entryIdx++;
    }
    if (entryIdx > 0) {
        args.entries = entries;
        args.num_entries = entryIdx;
    }
    amqp_queue_declare(conn, 1,
                       amqp_cstring_bytes(opt.queue.c_str()),
                       0, 1, 0, 0, args);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "queue.declare")) {
        amqp_connection_close(conn, AMQP_REPLY_SUCCESS);
        amqp_destroy_connection(conn);
        return false;
    }
    amqp_queue_bind(conn, 1,
                    amqp_cstring_bytes(opt.queue.c_str()),
                    amqp_cstring_bytes(opt.exchange.c_str()),
                    amqp_cstring_bytes(opt.bindingKey.c_str()),
                    amqp_empty_table);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "queue.bind")) {
        amqp_connection_close(conn, AMQP_REPLY_SUCCESS);
        amqp_destroy_connection(conn);
        return false;
    }

    amqp_basic_properties_t props;
    std::memset(&props, 0, sizeof(props));
    props._flags = AMQP_BASIC_CONTENT_TYPE_FLAG | AMQP_BASIC_DELIVERY_MODE_FLAG;
    props.content_type = amqp_cstring_bytes("application/json");
    props.delivery_mode = 2;
    std::string ttlStr = std::to_string(std::max(0, opt.messageTtlMs));
    if (opt.messageTtlMs > 0) {
        props._flags |= AMQP_BASIC_EXPIRATION_FLAG;
        props.expiration = amqp_cstring_bytes(ttlStr.c_str());
    }

    amqp_bytes_t body;
    body.len = payload.size();
    body.bytes = const_cast<char*>(payload.data());

    int rc = amqp_basic_publish(conn, 1,
                                amqp_cstring_bytes(opt.exchange.c_str()),
                                amqp_cstring_bytes(opt.routingKey.c_str()),
                                0, 0, &props, body);
    if (rc) {
        std::cerr << "ExternalSender: publish failed rc=" << rc << std::endl;
        amqp_connection_close(conn, AMQP_REPLY_SUCCESS);
        amqp_destroy_connection(conn);
        return false;
    }

    amqp_channel_close(conn, 1, AMQP_REPLY_SUCCESS);
    amqp_connection_close(conn, AMQP_REPLY_SUCCESS);
    amqp_destroy_connection(conn);
    return true;
}

}  // namespace ExternalSender

#endif  // EXTERNAL_SENDER_HELPERS_H
