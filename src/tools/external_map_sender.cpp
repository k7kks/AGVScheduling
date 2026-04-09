#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>

#include "common/PathUtils.h"
#include "common/RabbitMQConfig.h"
#include "nlohmann/json.hpp"
#include "external_sender_helpers.h"

int main() {
    RabbitMQConfig cfg = RabbitMQConfigFromEnv({});
    std::string exch = std::getenv("EXT_EXCHANGE") ? std::getenv("EXT_EXCHANGE") : cfg.dispToAlgoExchange;
    std::string queue = std::getenv("EXT_QUEUE") ? std::getenv("EXT_QUEUE") : cfg.dispToAlgoQueue;
    std::string bindingKey = std::getenv("EXT_BINDING_KEY") ? std::getenv("EXT_BINDING_KEY") : cfg.dispToAlgoBindingKey;
    const char* rkOverride = std::getenv("MAP_ROUTING_KEY");
    std::string routingKey = rkOverride && *rkOverride ? rkOverride : "SendMapInfo";
    if (const char* rkGlobal = std::getenv("EXT_ROUTING_KEY"); routingKey == "SendMapInfo" && rkGlobal && *rkGlobal) {
        routingKey = rkGlobal;
    }
    std::string exchType = std::getenv("EXT_EXCHANGE_TYPE") ? std::getenv("EXT_EXCHANGE_TYPE") : "fanout";
    int messageTtlMs = 0;
    if (const char* ttlEnv = std::getenv("EXT_MESSAGE_TTL_MS")) {
        try { messageTtlMs = std::max(0, std::stoi(ttlEnv)); } catch (...) {}
    }

    auto roots = PathUtils::commonRoots();
    PathUtils::addRootIfSet(roots, "AGV_SCHED_ROOT");
    std::string mapFile = PathUtils::resolvePathOrWarn(
        getenv("MAP_FILE") ? getenv("MAP_FILE") : "config/south_20260107.json",
        "map file", roots);

    std::ifstream ifs(mapFile);
    if (!ifs) {
        std::cerr << "external_map_sender: cannot open map file " << mapFile << std::endl;
        return 2;
    }
    nlohmann::json mapJson;
    try {
        ifs >> mapJson;
    } catch (const std::exception& e) {
        std::cerr << "external_map_sender: failed to parse map json: " << e.what() << std::endl;
        return 2;
    }

    const char* versionEnv = std::getenv("MAP_VERSION");
    std::string providedVersion = versionEnv && *versionEnv ? versionEnv : "demo-map";

    nlohmann::json message;
    if (mapJson.is_object() && mapJson.contains("mapData")) {
        message = mapJson;
        if (!message.contains("mapVersion") || message["mapVersion"].is_null()) {
            message["mapVersion"] = providedVersion;
        }
    } else if (mapJson.contains("nodes") || mapJson.contains("node")) {
        message["mapVersion"] = providedVersion;
        message["mapData"] = mapJson;
    } else {
        message["mapVersion"] = providedVersion;
        message["mapData"] = mapJson;
    }

    std::cout << "external_map_sender: exchange='" << exch
              << "' queue='" << queue
              << "' routingKey='" << routingKey
              << "' map='" << mapFile << "'\n";

    ExternalSender::PublishOptions opt;
    opt.exchange = exch;
    opt.exchangeType = exchType;
    opt.queue = queue;
    opt.routingKey = routingKey;
    opt.bindingKey = bindingKey.empty() ? "#" : bindingKey;
    opt.messageTtlMs = messageTtlMs;
    bool ok = ExternalSender::PublishPayload(cfg, opt, message.dump());
    return ok ? 0 : 2;
}
