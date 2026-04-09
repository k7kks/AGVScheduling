extern "C" {
#include <amqp.h>
#include <amqp_tcp_socket.h>
}

#include "message/TrafficPathPublisher.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "common/TaskFieldUtils.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

namespace {

static bool amqp_ok(amqp_rpc_reply_t r, const char* ctx) {
    if (r.reply_type != AMQP_RESPONSE_NORMAL) {
        std::cerr << "TrafficPathPublisher AMQP error in " << ctx
                  << ": reply_type=" << r.reply_type << std::endl;
        return false;
    }
    return true;
}

static double compute_yaw_deg(double dx, double dy) {
    constexpr double kPi = 3.14159265358979323846;
    double yaw = std::atan2(dy, dx) * 180.0 / kPi;
    if (yaw < 0) yaw += 360.0;
    return yaw;
}

static bool compute_segment_yaw(const MapInfo& mapInfo, int fromNodeId, int toNodeId, int& outYaw) {
    if (fromNodeId < 0 || toNodeId < 0 || fromNodeId == toNodeId) return false;
    try {
        const Node& fromNode = mapInfo.getNodeById(fromNodeId);
        const Node& toNode = mapInfo.getNodeById(toNodeId);
        double dx = toNode.x - fromNode.x;
        double dy = toNode.y - fromNode.y;
        if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) return false;
        int yaw = static_cast<int>(std::lround(compute_yaw_deg(dx, dy)));
        yaw %= 360;
        if (yaw < 0) yaw += 360;
        outYaw = yaw;
        return true;
    } catch (...) {
        return false;
    }
}

static int resolve_point_yaw(const MapInfo& mapInfo, const std::vector<int>& nodes, size_t idx) {
    int yaw = 0;
    int tmp = 0;
    if (idx + 1 < nodes.size() && compute_segment_yaw(mapInfo, nodes[idx], nodes[idx + 1], tmp)) {
        yaw = tmp;
    } else if (idx > 0 && compute_segment_yaw(mapInfo, nodes[idx - 1], nodes[idx], tmp)) {
        yaw = tmp;
    }
    return yaw;
}

static int parse_step_type_code(const std::string& stepType) {
    if (stepType.empty()) return -1;
    try {
        size_t consumed = 0;
        int code = std::stoi(stepType, &consumed);
        if (consumed > 0) return code;
    } catch (...) {
    }
    return TaskFieldUtils::PointTypeFromString(stepType);
}

static std::string node_id_to_string(int nodeId) {
    return nodeId >= 0 ? std::to_string(nodeId) : std::string("");
}

static json make_node_point(const MapInfo& mapInfo, int nodeId, int angleDeg, int pointType) {
    json pt;
    try {
        const Node& node = mapInfo.getNodeById(nodeId);
        pt["nodeId"] = node_id_to_string(nodeId);
        pt["x"] = static_cast<int>(std::lround(node.x));
        pt["y"] = static_cast<int>(std::lround(node.y));
    } catch (...) {
        pt["nodeId"] = std::string("");
        pt["x"] = 0;
        pt["y"] = 0;
    }
    pt["angle"] = angleDeg;
    pt["pointType"] = pointType;
    return pt;
}

static json make_path_point(const MapInfo& mapInfo, int nodeId, int yawDeg) {
    json pt;
    try {
        const Node& node = mapInfo.getNodeById(nodeId);
        pt["nodeId"] = node_id_to_string(nodeId);
        pt["x"] = static_cast<int>(std::lround(node.x));
        pt["y"] = static_cast<int>(std::lround(node.y));
    } catch (...) {
        pt["nodeId"] = std::string("");
        pt["x"] = 0;
        pt["y"] = 0;
    }
    pt["z"] = 0;
    pt["yaw"] = yawDeg;
    pt["curvature"] = 0;
    pt["distance"] = 0;
    pt["leftDistance"] = 0;
    pt["rightDistance"] = 0;
    pt["slope"] = 0;
    int speed = static_cast<int>(std::lround(mapInfo.getGlobalMaxSpeed()));
    pt["speed"] = speed;
    pt["maxSpeed"] = speed;
    return pt;
}

static std::vector<int> build_segment_route(const PathPlanningHelper::PathSegmentInfo& seg) {
    std::vector<int> route = seg.nodes;
    if (route.empty()) {
        if (seg.fromNodeId >= 0) route.push_back(seg.fromNodeId);
        if (seg.toNodeId >= 0 && (route.empty() || route.back() != seg.toNodeId)) {
            route.push_back(seg.toNodeId);
        }
    }
    return route;
}

static json build_segment_endpoint_point(const MapInfo& mapInfo,
                                         const PathPlanningHelper::PathSegmentInfo& seg,
                                         bool startPoint) {
    std::vector<int> route = build_segment_route(seg);
    if (route.empty()) {
        return make_node_point(mapInfo, -1, 0, startPoint ? -1 : parse_step_type_code(seg.stepType));
    }
    size_t idx = startPoint ? 0 : route.size() - 1;
    int yaw = resolve_point_yaw(mapInfo, route, idx);
    int pointType = startPoint ? -1 : parse_step_type_code(seg.stepType);
    return make_node_point(mapInfo, route[idx], yaw, pointType);
}

static json build_path_infos(const std::vector<PathPlanningHelper::AmrPlanInfo>& plans,
                             const MapInfo& mapInfo) {
    json pathInfos = json::array();
    for (const auto& plan : plans) {
        for (const auto& seg : plan.segments) {
            json info;
            info["deviceId"] = plan.amrId;
            info["startPoint"] = build_segment_endpoint_point(mapInfo, seg, true);
            info["endPoint"] = build_segment_endpoint_point(mapInfo, seg, false);
            json pathPoints = json::array();
            auto route = build_segment_route(seg);
            for (size_t i = 0; i < route.size(); ++i) {
                int nodeId = route[i];
                int yaw = resolve_point_yaw(mapInfo, route, i);
                pathPoints.push_back(make_path_point(mapInfo, nodeId, yaw));
            }
            info["pathPoints"] = std::move(pathPoints);
            pathInfos.push_back(std::move(info));
        }
    }
    return pathInfos;
}

}  // namespace

struct TrafficPathPublisher::Impl {
    explicit Impl(const RabbitMQConfig& cfgIn)
        : cfg(cfgIn),
          conn(nullptr),
          connected(false),
          channel_open(false),
          message_ttl_ms(0) {
        const char* e = std::getenv("TRAFFIC_PATH_EXCHANGE");
        const char* q = std::getenv("TRAFFIC_PATH_QUEUE");
        const char* r = std::getenv("TRAFFIC_PATH_ROUTING_KEY");
        const char* b = std::getenv("TRAFFIC_PATH_BINDING_KEY");
        exch = e ? e : cfg.algoToTrafficExchange;
        queue = q ? q : cfg.algoToTrafficQueue;
        rkey = r ? r : cfg.algoToTrafficRoutingKey;
        binding_key = b ? b : cfg.algoToTrafficBindingKey;
        if (binding_key.empty()) binding_key = "#";
        if (const char* ttlEnv = std::getenv("TRAFFIC_PATH_TTL_MS")) {
            try {
                int parsed = std::stoi(ttlEnv);
                if (parsed >= 0) message_ttl_ms = parsed;
            } catch (...) {
            }
        }
        expiration_str = std::to_string(std::max(0, message_ttl_ms));
    }

    ~Impl() {
        close();
    }

    bool ensure_connected() {
        if (connected && conn) return true;
        close();
        conn = amqp_new_connection();
        amqp_socket_t* sock = amqp_tcp_socket_new(conn);
        if (!sock) {
            std::cerr << "TrafficPathPublisher: cannot create TCP socket" << std::endl;
            close();
            return false;
        }
        if (amqp_socket_open(sock, cfg.host.c_str(), cfg.port)) {
            std::cerr << "TrafficPathPublisher: socket open failed" << std::endl;
            close();
            return false;
        }
        if (!amqp_ok(amqp_login(conn, cfg.vhost.c_str(), 0, 131072, 0,
                                AMQP_SASL_METHOD_PLAIN,
                                cfg.username.c_str(), cfg.password.c_str()),
                     "login")) {
            close();
            return false;
        }
        amqp_channel_open(conn, 1);
        if (!amqp_ok(amqp_get_rpc_reply(conn), "channel.open")) {
            close();
            return false;
        }
        channel_open = true;

        amqp_exchange_declare(conn, 1,
                              amqp_cstring_bytes(exch.c_str()),
                              amqp_cstring_bytes("fanout"),
                              0, 1, 0, 0, amqp_empty_table);
        if (!amqp_ok(amqp_get_rpc_reply(conn), "exchange.declare")) {
            close();
            return false;
        }
        amqp_table_entry_t qargs_entries[1];
        amqp_table_t qargs{0, nullptr};
        if (message_ttl_ms > 0) {
            qargs_entries[0].key = amqp_cstring_bytes("x-message-ttl");
            qargs_entries[0].value.kind = AMQP_FIELD_KIND_I32;
            qargs_entries[0].value.value.i32 = message_ttl_ms;
            qargs.entries = qargs_entries;
            qargs.num_entries = 1;
        }
        amqp_queue_declare(conn, 1,
                           amqp_cstring_bytes(queue.c_str()),
                           0, 1, 0, 0, qargs);
        if (!amqp_ok(amqp_get_rpc_reply(conn), "queue.declare")) {
            close();
            return false;
        }
        amqp_queue_bind(conn, 1,
                        amqp_cstring_bytes(queue.c_str()),
                        amqp_cstring_bytes(exch.c_str()),
                        amqp_cstring_bytes(binding_key.c_str()),
                        amqp_empty_table);
        if (!amqp_ok(amqp_get_rpc_reply(conn), "queue.bind")) {
            close();
            return false;
        }

        connected = true;
        return true;
    }

    bool publish(const std::string& payload) {
        if (!ensure_connected()) return false;
        amqp_bytes_t message_bytes;
        message_bytes.len = payload.size();
        message_bytes.bytes = (void*)payload.data();
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
                                    amqp_cstring_bytes(exch.c_str()),
                                    amqp_cstring_bytes(rkey.c_str()),
                                    0, 0, &props, message_bytes);
        if (rc != 0) {
            std::cerr << "TrafficPathPublisher: publish failed rc=" << rc << std::endl;
            close();
            return false;
        }
        return true;
    }

    void close() {
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

    RabbitMQConfig cfg;
    std::string exch;
    std::string queue;
    std::string rkey;
    std::string binding_key;
    int message_ttl_ms;
    std::string expiration_str;
    amqp_connection_state_t conn;
    bool connected;
    bool channel_open;
};

TrafficPathPublisher::TrafficPathPublisher(const RabbitMQConfig& cfg)
    : impl_(new Impl(cfg)) {}

TrafficPathPublisher::~TrafficPathPublisher() {
    delete impl_;
    impl_ = nullptr;
}

bool TrafficPathPublisher::publish(const std::string& messageId,
                                   const std::vector<PathPlanningHelper::AmrPlanInfo>& plans,
                                   const MapInfo& mapInfo,
                                   int code,
                                   const std::string& error) {
    if (!impl_) return false;
    json body;
    body["messageId"] = messageId;
    body["code"] = code;
    body["error"] = error;
    body["pathInfos"] = build_path_infos(plans, mapInfo);
    return impl_->publish(body.dump());
}

bool TrafficPathPublisher::publishJson(const nlohmann::json& body) {
    if (!impl_) return false;
    return impl_->publish(body.dump());
}

bool TrafficPathPublisher::publishPayload(const std::string& payload) {
    if (!impl_) return false;
    return impl_->publish(payload);
}
