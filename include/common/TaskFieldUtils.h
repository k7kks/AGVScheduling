#ifndef TASK_FIELD_UTILS_H
#define TASK_FIELD_UTILS_H

#include <algorithm>
#include <cctype>
#include <string>

namespace TaskFieldUtils {

enum : int {
    REQUEST_TYPE_UNKNOWN = -1,
    REQUEST_TYPE_BATCH = 0,
    REQUEST_TYPE_SINGLE = 1,
    REQUEST_TYPE_URGENT = 2
};

enum : int {
    POINT_TYPE_UNKNOWN = -1,
    POINT_TYPE_PICKUP = 0,
    POINT_TYPE_DROPOFF = 1,
    POINT_TYPE_WAYPOINT = 2,
    POINT_TYPE_CHARGE = 3,
    POINT_TYPE_STANDBY = 4,
    POINT_TYPE_MAINTENANCE = 5
};

inline std::string NormalizeAscii(std::string s) {
    std::string out;
    out.reserve(s.size());
    for (unsigned char ch : s) {
        if (!std::isspace(ch)) {
            out.push_back(static_cast<char>(std::toupper(ch)));
        }
    }
    return out;
}

inline int RequestTypeFromString(const std::string& text) {
    if (text.empty()) return REQUEST_TYPE_UNKNOWN;
    std::string norm = NormalizeAscii(text);
    if (norm == "BATCH" || norm == "批量" || norm == "0") {
        return REQUEST_TYPE_BATCH;
    }
    if (norm == "SINGLE" || norm == "单任务" || norm == "1") {
        return REQUEST_TYPE_SINGLE;
    }
    if (norm == "URGENT" || norm == "紧急调度" || norm == "2") {
        return REQUEST_TYPE_URGENT;
    }
    return REQUEST_TYPE_UNKNOWN;
}

inline std::string DescribeRequestType(int code) {
    switch (code) {
        case REQUEST_TYPE_BATCH: return "BATCH";
        case REQUEST_TYPE_SINGLE: return "SINGLE";
        case REQUEST_TYPE_URGENT: return "URGENT";
        default: return "UNKNOWN";
    }
}

inline int PointTypeFromString(const std::string& text) {
    if (text.empty()) return POINT_TYPE_UNKNOWN;
    std::string norm = NormalizeAscii(text);
    if (norm == "PICKUP" || norm == "取货") return POINT_TYPE_PICKUP;
    if (norm == "DROPOFF" || norm == "DELIVERY" || norm == "放货") return POINT_TYPE_DROPOFF;
    if (norm == "WAYPOINT" || norm == "WAY" || norm == "经过点") return POINT_TYPE_WAYPOINT;
    if (norm == "CHARGE" || norm == "CHARGING" || norm == "充电") return POINT_TYPE_CHARGE;
    if (norm == "STANDBY" || norm == "WAIT" || norm == "待命") return POINT_TYPE_STANDBY;
    if (norm == "MAINTENANCE" || norm == "MAINTENANCE_CALL" || norm == "维护") return POINT_TYPE_MAINTENANCE;
    return POINT_TYPE_UNKNOWN;
}

inline std::string DescribePointType(int code) {
    switch (code) {
        case POINT_TYPE_PICKUP: return "PICKUP";
        case POINT_TYPE_DROPOFF: return "DROPOFF";
        case POINT_TYPE_WAYPOINT: return "WAYPOINT";
        case POINT_TYPE_CHARGE: return "CHARGE";
        case POINT_TYPE_STANDBY: return "STANDBY";
        case POINT_TYPE_MAINTENANCE: return "MAINTENANCE";
        default: return "UNKNOWN";
    }
}

}  // namespace TaskFieldUtils

#endif  // TASK_FIELD_UTILS_H
