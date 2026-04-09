#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include "common/RabbitMQConfig.h"
#include "common/PathUtils.h"
#include "nlohmann/json.hpp"
#include "external_sender_helpers.h"

namespace {

std::string LoadPayload(const std::string& filePath) {
    std::ifstream ifs(filePath);
    if (!ifs) {
        throw std::runtime_error("external_status_sender: cannot open payload file " + filePath);
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
    const char* rkOverride = std::getenv("STATUS_ROUTING_KEY");
    std::string routingKey = rkOverride && *rkOverride ? rkOverride : "SendRobotStatusInfos";
    if (const char* rkGlobal = std::getenv("EXT_ROUTING_KEY"); routingKey == "SendRobotStatusInfos" && rkGlobal && *rkGlobal) {
        routingKey = rkGlobal;
    }
    std::string exchType = std::getenv("EXT_EXCHANGE_TYPE") ? std::getenv("EXT_EXCHANGE_TYPE") : "fanout";
    int messageTtlMs = 0;
    if (const char* ttlEnv = std::getenv("EXT_MESSAGE_TTL_MS")) {
        try { messageTtlMs = std::max(0, std::stoi(ttlEnv)); } catch (...) {}
    }

    // Require an external file; default to the simple map status sample
    const char* payloadEnv = std::getenv("STATUS_PAYLOAD_FILE");
    std::string payloadPath = payloadEnv && *payloadEnv
        ? payloadEnv
        : "script/status_simple_warehouse.json";

    auto roots = PathUtils::commonRoots();
    PathUtils::addRootIfSet(roots, "AGV_SCHED_ROOT");
    payloadPath = PathUtils::resolvePathOrWarn(payloadPath, "status payload", roots);

    std::string payload;
    try {
        payload = LoadPayload(payloadPath);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 2;
    }

    // 兼容两种格式：若顶层为对象并包含 robotStatusInfos，则仅发送数组部分
    std::string finalPayload = payload;
    try {
        nlohmann::json parsed = nlohmann::json::parse(payload);
        const nlohmann::json* statusArray = nullptr;
        if (parsed.is_array()) {
            statusArray = &parsed;
        } else if (parsed.contains("robotStatusInfos") && parsed["robotStatusInfos"].is_array()) {
            statusArray = &parsed["robotStatusInfos"];
        } else if (parsed.contains("RobotStatusInfos") && parsed["RobotStatusInfos"].is_array()) {
            statusArray = &parsed["RobotStatusInfos"];
        }
        if (statusArray) {
            finalPayload = statusArray->dump();
        }
    } catch (const std::exception& ex) {
        std::cerr << "external_status_sender: payload parse failed (" << ex.what()
                  << "), sending raw text." << std::endl;
    }

    std::cout << "external_status_sender: exchange='" << exch
              << "' queue='" << queue
              << "' routingKey='" << routingKey
              << "' host=" << cfg.host << ":" << cfg.port
              << " payload='" << payloadPath << "'" << std::endl;

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
