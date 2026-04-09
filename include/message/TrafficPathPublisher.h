// Algo -> Traffic: publish aggregated path info after assignment
#ifndef TRAFFIC_PATH_PUBLISHER_H
#define TRAFFIC_PATH_PUBLISHER_H

#include <string>
#include <vector>

#include "common/RabbitMQConfig.h"
#include "data/MapInfo.h"
#include "nlohmann/json.hpp"
#include "path_planning/PathPlanningHelper.h"

class TrafficPathPublisher {
public:
    explicit TrafficPathPublisher(const RabbitMQConfig& cfg);
    ~TrafficPathPublisher();

    TrafficPathPublisher(const TrafficPathPublisher&) = delete;
    TrafficPathPublisher& operator=(const TrafficPathPublisher&) = delete;

    bool publish(const std::string& messageId,
                 const std::vector<PathPlanningHelper::AmrPlanInfo>& plans,
                 const MapInfo& mapInfo,
                 int code = 200,
                 const std::string& error = "");
    bool publishJson(const nlohmann::json& body);
    bool publishPayload(const std::string& payload);

private:
    struct Impl;
    Impl* impl_;
};

#endif  // TRAFFIC_PATH_PUBLISHER_H
