// AlgoPublisher: send algo->business MQ messages (init, heartbeat, path response, etc.)
#ifndef ALGO_PUBLISHER_H
#define ALGO_PUBLISHER_H

#include <string>
#include "common/RabbitMQConfig.h"
#include "nlohmann/json.hpp"

class AlgoPublisher {
public:
    explicit AlgoPublisher(const RabbitMQConfig& cfg);
    ~AlgoPublisher();

    AlgoPublisher(const AlgoPublisher&) = delete;
    AlgoPublisher& operator=(const AlgoPublisher&) = delete;

    bool publishJson(const std::string& routingKey, const nlohmann::json& body);

    bool sendServiceInitialize(const std::string& serviceName);
    bool sendHeartbeat(const std::string& serviceName, bool allocating = false);
    bool sendRunInfo(const std::string& deviceId,
                     const std::string& desc,
                     const std::string& timeStr);
    bool sendAlertInfo(const std::string& deviceId,
                       int alertCode,
                       const std::string& alertLevel,
                       const std::string& alertDesc,
                       const std::string& timeStr);
    bool sendPathResponse(const nlohmann::json& body);
    bool sendAroundPathResponse(const nlohmann::json& body);
    bool sendTrailResponse(const nlohmann::json& body);

private:
    struct Impl;
    Impl* impl_;
};

#endif  // ALGO_PUBLISHER_H
