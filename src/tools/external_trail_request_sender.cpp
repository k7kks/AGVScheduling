#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>

#include "common/RabbitMQConfig.h"
#include "nlohmann/json.hpp"
#include "external_sender_helpers.h"

static std::string BuildMessageId(const char* envVar, const std::string& prefix) {
    if (envVar && *envVar) return std::string(envVar);
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    std::ostringstream oss;
    oss << prefix << "_" << ms;
    return oss.str();
}

int main() {
    RabbitMQConfig cfg = RabbitMQConfigFromEnv({});
    std::string exch = std::getenv("EXT_EXCHANGE") ? std::getenv("EXT_EXCHANGE") : cfg.dispToAlgoExchange;
    std::string queue = std::getenv("EXT_QUEUE") ? std::getenv("EXT_QUEUE") : cfg.dispToAlgoQueue;
    std::string bindingKey = std::getenv("EXT_BINDING_KEY") ? std::getenv("EXT_BINDING_KEY") : cfg.dispToAlgoBindingKey;
    const char* rkOverride = std::getenv("TRAIL_REQUEST_ROUTING_KEY");
    std::string routingKey = rkOverride && *rkOverride ? rkOverride : "RobotTrailRequest";
    if (const char* rkGlobal = std::getenv("EXT_ROUTING_KEY"); routingKey == "RobotTrailRequest" && rkGlobal && *rkGlobal) {
        routingKey = rkGlobal;
    }
    std::string exchType = std::getenv("EXT_EXCHANGE_TYPE") ? std::getenv("EXT_EXCHANGE_TYPE") : "fanout";
    int messageTtlMs = 0;
    if (const char* ttlEnv = std::getenv("EXT_MESSAGE_TTL_MS")) {
        try { messageTtlMs = std::max(0, std::stoi(ttlEnv)); } catch (...) {}
    }

    std::string messageId = BuildMessageId(std::getenv("TRAIL_REQUEST_MESSAGE_ID"), "TRAIL_REQ");
    std::string deviceId = std::getenv("TRAIL_REQUEST_DEVICE_ID") ? std::getenv("TRAIL_REQUEST_DEVICE_ID") : "AGV01";
    const char* subTaskIdEnv = std::getenv("TRAIL_REQUEST_SUBTASK_ID");
    int mapId = 0;
    if (const char* mapEnv = std::getenv("TRAIL_REQUEST_MAP_ID")) {
        try { mapId = std::max(0, std::stoi(mapEnv)); } catch (...) { mapId = 0; }
    }

    nlohmann::json body = {
        {"messageId", messageId},
        {"deviceId", deviceId}
    };
    if (mapId > 0) {
        body["mapId"] = mapId;
    }
    if (subTaskIdEnv && *subTaskIdEnv) {
        body["subTaskId"] = std::string(subTaskIdEnv);
    }

    std::cout << "external_trail_request_sender: deviceId=" << deviceId
              << " routingKey=" << routingKey;
    if (mapId > 0) {
        std::cout << " mapId=" << mapId;
    }
    if (subTaskIdEnv && *subTaskIdEnv) {
        std::cout << " subTaskId=" << subTaskIdEnv;
    }
    std::cout << std::endl;

    ExternalSender::PublishOptions opt;
    opt.exchange = exch;
    opt.exchangeType = exchType;
    opt.queue = queue;
    opt.routingKey = routingKey;
    opt.bindingKey = bindingKey.empty() ? "#" : bindingKey;
    opt.messageTtlMs = messageTtlMs;
    bool ok = ExternalSender::PublishPayload(cfg, opt, body.dump());
    return ok ? 0 : 2;
}
