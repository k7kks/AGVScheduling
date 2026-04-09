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
    const char* rkOverride = std::getenv("PATH_REQUEST_ROUTING_KEY");
    std::string routingKey = rkOverride && *rkOverride ? rkOverride : "RobotPathRequest";
    if (const char* rkGlobal = std::getenv("EXT_ROUTING_KEY"); routingKey == "RobotPathRequest" && rkGlobal && *rkGlobal) {
        routingKey = rkGlobal;
    }
    std::string exchType = std::getenv("EXT_EXCHANGE_TYPE") ? std::getenv("EXT_EXCHANGE_TYPE") : "fanout";
    int messageTtlMs = 0;
    if (const char* ttlEnv = std::getenv("EXT_MESSAGE_TTL_MS")) {
        try { messageTtlMs = std::max(0, std::stoi(ttlEnv)); } catch (...) {}
    }

    std::string messageId = BuildMessageId(std::getenv("PATH_REQUEST_MESSAGE_ID"), "PATH_REQ");
    std::string deviceId = std::getenv("PATH_REQUEST_DEVICE_ID") ? std::getenv("PATH_REQUEST_DEVICE_ID") : "AGV01";
    int mapId = 0;
    if (const char* mapEnv = std::getenv("PATH_REQUEST_MAP_ID")) {
        try { mapId = std::max(0, std::stoi(mapEnv)); } catch (...) { mapId = 0; }
    }

    nlohmann::json body = {
        {"messageId", messageId},
        {"deviceId", deviceId}
    };
    if (mapId > 0) {
        body["mapId"] = mapId;
    }

    std::cout << "external_path_request_sender: deviceId=" << deviceId
              << " routingKey=" << routingKey;
    if (mapId > 0) {
        std::cout << " mapId=" << mapId;
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
