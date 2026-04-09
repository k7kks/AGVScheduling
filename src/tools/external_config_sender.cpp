#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include "common/RabbitMQConfig.h"
#include "external_sender_helpers.h"
#include "nlohmann/json.hpp"

namespace {

std::string LoadPayload(const char* filePath, const std::string& fallback) {
    if (!filePath || !*filePath) return fallback;
    std::ifstream ifs(filePath);
    if (!ifs) {
        std::cerr << "external_config_sender: cannot open payload file " << filePath
                  << ", use default sample\n";
        return fallback;
    }
    return std::string((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());
}

}  // namespace

int main() {
    RabbitMQConfig cfg = RabbitMQConfigFromEnv({});
    std::string exch = std::getenv("EXT_EXCHANGE") ? std::getenv("EXT_EXCHANGE") : cfg.dispToAlgoExchange;
    std::string queue = std::getenv("EXT_QUEUE") ? std::getenv("EXT_QUEUE") : cfg.dispToAlgoQueue;
    std::string bindingKey = std::getenv("EXT_BINDING_KEY") ? std::getenv("EXT_BINDING_KEY") : cfg.dispToAlgoBindingKey;
    const char* rkOverride = std::getenv("CONFIG_ROUTING_KEY");
    std::string routingKey = rkOverride && *rkOverride ? rkOverride : "SendRobotConfigInfos";
    if (const char* rkGlobal = std::getenv("EXT_ROUTING_KEY"); routingKey == "SendRobotConfigInfos" && rkGlobal && *rkGlobal) {
        routingKey = rkGlobal;
    }
    std::string exchType = std::getenv("EXT_EXCHANGE_TYPE") ? std::getenv("EXT_EXCHANGE_TYPE") : "fanout";
    int messageTtlMs = 0;
    if (const char* ttlEnv = std::getenv("EXT_MESSAGE_TTL_MS")) {
        try { messageTtlMs = std::max(0, std::stoi(ttlEnv)); } catch (...) {}
    }

    static const std::string defaultPayload = R"JSON({
  "robotConfigInfos": [
    { "agv_id": "AGV01", "agv_type": "FORK", "length": 2100, "width": 900 },
    { "agv_id": "AGV02", "agv_type": "PULLER", "length": 1800, "width": 800 },
    { "agv_id": "AGV03", "agv_type": "FORK", "length": 2100, "width": 900 },
    { "agv_id": "AGV04", "agv_type": "PULLER", "length": 1800, "width": 800 },
    { "agv_id": "AGV05", "agv_type": "FORK", "length": 2100, "width": 900 },
    { "agv_id": "AGV06", "agv_type": "PULLER", "length": 1800, "width": 800 },
    { "agv_id": "AGV07", "agv_type": "FORK", "length": 2100, "width": 900 },
    { "agv_id": "AGV08", "agv_type": "PULLER", "length": 1800, "width": 800 },
    { "agv_id": "AGV09", "agv_type": "FORK", "length": 2100, "width": 900 },
    { "agv_id": "AGV10", "agv_type": "PULLER", "length": 1800, "width": 800 },
    { "agv_id": "AGV11", "agv_type": "FORK", "length": 2100, "width": 900 },
    { "agv_id": "AGV12", "agv_type": "PULLER", "length": 1800, "width": 800 },
    { "agv_id": "AGV13", "agv_type": "FORK", "length": 2100, "width": 900 },
    { "agv_id": "AGV14", "agv_type": "PULLER", "length": 1800, "width": 800 },
    { "agv_id": "AGV15", "agv_type": "FORK", "length": 2100, "width": 900 },
    { "agv_id": "AGV16", "agv_type": "PULLER", "length": 1800, "width": 800 },
    { "agv_id": "AGV17", "agv_type": "FORK", "length": 2100, "width": 900 },
    { "agv_id": "AGV18", "agv_type": "PULLER", "length": 1800, "width": 800 },
    { "agv_id": "AGV19", "agv_type": "FORK", "length": 2100, "width": 900 },
    { "agv_id": "AGV20", "agv_type": "PULLER", "length": 1800, "width": 800 }
  ]
})JSON";

    std::string payload = LoadPayload(std::getenv("CONFIG_PAYLOAD_FILE"), defaultPayload.c_str());

    std::string finalPayload = payload;
    try {
        nlohmann::json parsed = nlohmann::json::parse(payload);
        const nlohmann::json* configArray = nullptr;
        if (parsed.is_array()) {
            configArray = &parsed;
        } else if (parsed.contains("robotConfigInfos") && parsed["robotConfigInfos"].is_array()) {
            configArray = &parsed["robotConfigInfos"];
        } else if (parsed.contains("RobotConfigInfos") && parsed["RobotConfigInfos"].is_array()) {
            configArray = &parsed["RobotConfigInfos"];
        }
        if (configArray) {
            finalPayload = configArray->dump();
        }
    } catch (const std::exception& ex) {
        std::cerr << "external_config_sender: payload parse failed (" << ex.what()
                  << "), sending raw text." << std::endl;
    }

    std::cout << "external_config_sender: exchange='" << exch
              << "' queue='" << queue
              << "' routingKey='" << routingKey
              << "' host=" << cfg.host << ":" << cfg.port << std::endl;

    ExternalSender::PublishOptions opt;
    opt.exchange = exch;
    opt.exchangeType = exchType;
    opt.queue = queue;
    opt.routingKey = routingKey;
    opt.bindingKey = bindingKey.empty() ? "#" : bindingKey;
    opt.messageTtlMs = messageTtlMs;
    bool ok = ExternalSender::PublishPayload(cfg, opt, finalPayload);
    return ok ? 0 : 2;
}
