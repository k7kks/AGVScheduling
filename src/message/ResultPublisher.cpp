extern "C" {
#include <amqp.h>
#include <amqp_tcp_socket.h>
}

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <cctype>
#include <fstream>
#include <algorithm>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include "message/ResultPublisher.h"
#include "nlohmann/json.hpp"
#include "GlobalPathPlanner.h"
#include "StaticPathTable.h"
#include "AStarPathFinder.h"
#include "algorithm/ShortestPathUpdater.h"
#include "path_planning/PathPlanningHelper.h"
#include "common/PathUtils.h"
#include "common/PathPlanningConstants.h"
#include "common/TaskFieldUtils.h"

using json = nlohmann::json;

// 前置声明：时间格式化工具
static std::string iso8601_utc_now();
static std::string iso8601_utc_after_seconds(int seconds);
static std::string filename_timestamp_local();
static void dump_result_to_file(const std::string& payload,
                                const std::string& directory);

// 环境变量/文件工具
static std::string getenv_str(const char* key, const char* defv) {
    if (const char* v = std::getenv(key)) return std::string(v);
    return std::string(defv);
}

static int getenv_int(const char* key, int defv) {
    if (const char* v = std::getenv(key)) {
        try { return std::stoi(v); } catch (...) { return defv; }
    }
    return defv;
}

static double getenv_double(const char* key, double defv) {
    if (const char* v = std::getenv(key)) {
        try { return std::stod(v); } catch (...) { return defv; }
    }
    return defv;
}

// JSON 帮助函数：点/节点解析
static std::string node_id_to_string(int nodeId) {
    return nodeId >= 0 ? std::to_string(nodeId) : std::string("");
}

static json make_point_json(int x, int y, int nodeId) {
    json j;
    j["x"] = x;
    j["y"] = y;
    j["nodeId"] = node_id_to_string(nodeId);
    return j;
}

static json make_point_json_from_point(const Point& p) {
    return make_point_json(p.getX(), p.getY(), p.getNodeId());
}

static int parse_node_id(const json& point) {
    if (!point.is_object()) return -1;
    if (!point.contains("nodeId")) return -1;
    const auto& nid = point["nodeId"];
    try {
        if (nid.is_number_integer()) {
            return nid.get<int>();
        } else if (nid.is_string()) {
            std::string s = nid.get<std::string>();
            if (s.empty()) return -1;
            return std::stoi(s);
        }
    } catch (...) {
        return -1;
    }
    return -1;
}

static json to_node_array(const std::vector<int>& nodes) {
    json arr = json::array();
    for (int nodeId : nodes) {
        arr.push_back(std::to_string(nodeId));
    }
    return arr;
}

static double pathresp_compute_yaw_deg(double dx, double dy) {
    constexpr double kPi = 3.14159265358979323846;
    double yaw = std::atan2(dy, dx) * 180.0 / kPi;
    if (yaw < 0) yaw += 360.0;
    return yaw;
}

static bool pathresp_compute_segment_yaw(const MapInfo& mapInfo,
                                         int fromNodeId,
                                         int toNodeId,
                                         int& outYaw) {
    if (fromNodeId < 0 || toNodeId < 0 || fromNodeId == toNodeId) return false;
    try {
        const Node& fromNode = mapInfo.getNodeById(fromNodeId);
        const Node& toNode = mapInfo.getNodeById(toNodeId);
        double dx = toNode.x - fromNode.x;
        double dy = toNode.y - fromNode.y;
        if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) return false;
        int yaw = static_cast<int>(std::lround(pathresp_compute_yaw_deg(dx, dy)));
        yaw %= 360;
        if (yaw < 0) yaw += 360;
        outYaw = yaw;
        return true;
    } catch (...) {
        return false;
    }
}

static int pathresp_resolve_point_yaw(const MapInfo& mapInfo,
                                      const std::vector<int>& nodes,
                                      size_t idx) {
    int yaw = 0;
    int tmp = 0;
    if (idx + 1 < nodes.size() && pathresp_compute_segment_yaw(mapInfo, nodes[idx], nodes[idx + 1], tmp)) {
        yaw = tmp;
    } else if (idx > 0 && pathresp_compute_segment_yaw(mapInfo, nodes[idx - 1], nodes[idx], tmp)) {
        yaw = tmp;
    }
    return yaw;
}

static int pathresp_parse_step_type_code(const std::string& stepType) {
    if (stepType.empty()) return -1;
    try {
        size_t consumed = 0;
        int code = std::stoi(stepType, &consumed);
        if (consumed > 0) return code;
    } catch (...) {}
    return TaskFieldUtils::PointTypeFromString(stepType);
}

static json pathresp_make_node_point(const MapInfo& mapInfo,
                                     int nodeId,
                                     int yawDeg,
                                     int pointType) {
    json pt;
    try {
        const Node& node = mapInfo.getNodeById(nodeId);
        pt["nodeId"] = node_id_to_string(nodeId);
        pt["x"] = static_cast<int>(std::lround(node.x));
        pt["y"] = static_cast<int>(std::lround(node.y));
    } catch (...) {
        pt["nodeId"] = node_id_to_string(nodeId);
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
    pt["pointType"] = pointType;
    return pt;
}

static json pathresp_make_path_point(const MapInfo& mapInfo, int nodeId, int yawDeg) {
    json pt;
    try {
        const Node& node = mapInfo.getNodeById(nodeId);
        pt["nodeId"] = node_id_to_string(nodeId);
        pt["x"] = static_cast<int>(std::lround(node.x));
        pt["y"] = static_cast<int>(std::lround(node.y));
    } catch (...) {
        pt["nodeId"] = node_id_to_string(nodeId);
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

static void pathresp_dedup_consecutive_nodes(std::vector<int>& nodes) {
    if (nodes.size() < 2) return;
    std::vector<int> dedup;
    dedup.reserve(nodes.size());
    for (int n : nodes) {
        if (dedup.empty() || dedup.back() != n) dedup.push_back(n);
    }
    nodes.swap(dedup);
}

static std::optional<int> pathresp_pick_any_neighbor_node_id(const MapInfo& mapInfo, int nodeId) {
    if (nodeId < 0) return std::nullopt;
    const auto& id2index = mapInfo.getId2Index();
    auto it = id2index.find(nodeId);
    if (it == id2index.end()) return std::nullopt;
    int nodeIndex = it->second;
    auto pick_from = [&](const std::unordered_map<int, std::vector<int>>& table) -> std::optional<int> {
        auto jt = table.find(nodeIndex);
        if (jt == table.end()) return std::nullopt;
        const auto& neighbors = jt->second;
        for (int nbIndex : neighbors) {
            try {
                const Node& nb = mapInfo.getNode(nbIndex);
                if (nb.id >= 0 && nb.id != nodeId) return nb.id;
            } catch (...) {
            }
        }
        return std::nullopt;
    };
    if (auto nb = pick_from(mapInfo.getAftNode())) return nb;
    if (auto nb = pick_from(mapInfo.getPreNode())) return nb;
    return std::nullopt;
}

static bool pathresp_normalize_route_min_nodes(const MapInfo& mapInfo,
                                               std::vector<int>& route,
                                               int fallbackNodeId,
                                               int minNodes) {
    if (route.empty() && fallbackNodeId >= 0) {
        route.push_back(fallbackNodeId);
    }
    for (int nodeId : route) {
        if (nodeId < 0) return false;
    }
    pathresp_dedup_consecutive_nodes(route);
    if (route.empty()) return false;
    while (static_cast<int>(route.size()) < minNodes) {
        int last = route.back();
        auto nb = pathresp_pick_any_neighbor_node_id(mapInfo, last);
        if (!nb.has_value() || *nb == last) return false;
        route.push_back(*nb);
        pathresp_dedup_consecutive_nodes(route);
        if (route.empty()) return false;
    }
    return true;
}

static json pathresp_build_segment_endpoint_point(const MapInfo& mapInfo,
                                                  const PathPlanningHelper::PathSegmentInfo& seg,
                                                  bool startPoint) {
    std::vector<int> route = seg.nodes;
    if (route.empty()) {
        if (seg.fromNodeId >= 0) route.push_back(seg.fromNodeId);
        if (seg.toNodeId >= 0 && (route.empty() || route.back() != seg.toNodeId)) {
            route.push_back(seg.toNodeId);
        }
    }
    if (route.empty()) route.push_back(-1);
    size_t idx = startPoint ? 0 : route.size() - 1;
    int yaw = pathresp_resolve_point_yaw(mapInfo, route, idx);
    int pointType = startPoint ? -1 : pathresp_parse_step_type_code(seg.stepType);
    return pathresp_make_node_point(mapInfo, route[idx], yaw, pointType);
}

static json build_path_response_body(const std::string& messageId,
                                     const std::string& deviceId,
                                     const PathPlanningHelper::AmrPlanInfo& plan,
                                     const MapInfo& mapInfo) {
    json resp;
    resp["messageId"] = messageId;
    resp["deviceId"] = deviceId;
    resp["code"] = 200;
    resp["error"] = "";
    json pathInfos = json::array();
    bool valid = true;
    for (const auto& seg : plan.segments) {
        json info;
        json pathPoints = json::array();
        std::vector<int> route = seg.nodes;
        if (route.empty()) {
            if (seg.fromNodeId >= 0) route.push_back(seg.fromNodeId);
            if (seg.toNodeId >= 0 && (route.empty() || route.back() != seg.toNodeId)) {
                route.push_back(seg.toNodeId);
            }
        }
        if (!pathresp_normalize_route_min_nodes(mapInfo, route, seg.fromNodeId, 2)) {
            valid = false;
            break;
        }
        PathPlanningHelper::PathSegmentInfo segResp = seg;
        segResp.nodes = route;
        segResp.fromNodeId = route.front();
        segResp.toNodeId = route.back();
        info["startPoint"] = pathresp_build_segment_endpoint_point(mapInfo, segResp, true);
        info["endPoint"] = pathresp_build_segment_endpoint_point(mapInfo, segResp, false);
        for (size_t i = 0; i < route.size(); ++i) {
            int nodeId = route[i];
            int yaw = pathresp_resolve_point_yaw(mapInfo, route, i);
            pathPoints.push_back(pathresp_make_path_point(mapInfo, nodeId, yaw));
        }
        info["pathPoints"] = std::move(pathPoints);
        pathInfos.push_back(std::move(info));
    }
    if (!valid) {
        resp["code"] = 422;
        resp["error"] = "Invalid path response route";
        resp["pathInfos"] = json::array();
        return resp;
    }
    resp["pathInfos"] = std::move(pathInfos);
    return resp;
}

static json build_path_response_map(const std::vector<Amr>& amrList,
                                    const std::vector<PathPlanningHelper::AmrPlanInfo>& plans,
                                    const MapInfo& mapInfo,
                                    const std::string& messageId) {
    std::unordered_map<std::string, const PathPlanningHelper::AmrPlanInfo*> planLookup;
    planLookup.reserve(plans.size());
    for (const auto& plan : plans) {
        planLookup[plan.amrId] = &plan;
    }
    json out = json::object();
    for (const auto& amr : amrList) {
        std::string deviceId = amr.getDeviceId();
        if (deviceId.empty()) continue;
        auto it = planLookup.find(deviceId);
        if (it == planLookup.end()) {
            json resp;
            resp["messageId"] = messageId;
            resp["deviceId"] = deviceId;
            resp["code"] = 404;
            resp["error"] = "No cached plan for device";
            resp["pathInfos"] = json::array();
            out[deviceId] = std::move(resp);
            continue;
        }
        out[deviceId] = build_path_response_body(messageId, deviceId, *(it->second), mapInfo);
    }
    return out;
}

static std::string path_source_to_string(GlobalPathPlanner::PathSource src) {
    switch (src) {
        case GlobalPathPlanner::PathSource::STATIC_TABLE:   return "STATIC_TABLE";
        case GlobalPathPlanner::PathSource::MIXED_PLANNING: return "MIXED_PLANNING";
        case GlobalPathPlanner::PathSource::PURE_ASTAR:     return "PURE_ASTAR";
        case GlobalPathPlanner::PathSource::UNREACHABLE:    return "UNREACHABLE";
        default:                                            return "UNKNOWN";
    }
}

static std::string infer_task_type_str(const Task& t) {
    // 约定：taskType==3 为 CHARGE
    if (t.getTaskType() == 3) return "CHARGE";
    bool hasPickup=false, hasDropoff=false, hasStandby=false, hasMaintenance=false, hasCharge=false;
    for (const auto& st : t.getSubTasks()) {
        int pt = st.getPointType();
        if (pt == TaskFieldUtils::POINT_TYPE_PICKUP) hasPickup = true;
        else if (pt == TaskFieldUtils::POINT_TYPE_DROPOFF) hasDropoff = true;
        else if (pt == TaskFieldUtils::POINT_TYPE_STANDBY) hasStandby = true;
        else if (pt == TaskFieldUtils::POINT_TYPE_MAINTENANCE) hasMaintenance = true;
        else if (pt == TaskFieldUtils::POINT_TYPE_CHARGE) hasCharge = true;
    }
    if (hasCharge) return "CHARGE";
    if (hasPickup && hasDropoff) return "MATERIAL_TRANSPORT";
    if (hasPickup && !hasDropoff) return "CALL_FOR_PICKUP";
    if (hasStandby) return "STANDBY";
    if (hasMaintenance) return "MAINTENANCE_CALL";
    return std::string("");
}

static void fill_task_type_specific(const Task& t, json& taskJson) {
    const auto& subs = t.getSubTasks();
    auto typeStr = infer_task_type_str(t);
    if (typeStr == "MATERIAL_TRANSPORT") {
        // 选取第一个PICKUP和第一个DROPOFF
        const SubTask* pickup = nullptr; const SubTask* dropoff = nullptr;
        for (const auto& st : subs) {
            int pt = st.getPointType();
            if (!pickup && pt == TaskFieldUtils::POINT_TYPE_PICKUP) pickup=&st;
            if (!dropoff && pt == TaskFieldUtils::POINT_TYPE_DROPOFF) dropoff=&st;
        }
        if (pickup) taskJson["pickup_point"] = make_point_json_from_point(pickup->getPoint()); else taskJson["pickup_point"] = make_point_json(0,0,-1);
        if (dropoff) taskJson["delivery_point"] = make_point_json_from_point(dropoff->getPoint()); else taskJson["delivery_point"] = make_point_json(0,0,-1);
        // 物料信息默认占位
        taskJson["material_info"] = { {"material_id",""}, {"quantity",0}, {"container_type",""} };
    } else if (typeStr == "CALL_FOR_PICKUP") {
        // 目标点优先取PICKUP，否则首个子任务
        const SubTask* tgt = nullptr;
        for (const auto& st : subs) {
            if (st.getPointType() == TaskFieldUtils::POINT_TYPE_PICKUP) { tgt=&st; break; }
        }
        if (!tgt && !subs.empty()) tgt = &subs.front();
        if (tgt) taskJson["target_point"] = make_point_json_from_point(tgt->getPoint()); else taskJson["target_point"] = make_point_json(0,0,-1);
        taskJson["wait_timeout"] = 0;
        taskJson["call_reason"] = "";
    } else if (typeStr == "CHARGE") {
        const SubTask* tgt = nullptr;
        for (const auto& st : subs) {
            if (st.getPointType() == TaskFieldUtils::POINT_TYPE_CHARGE) { tgt=&st; break; }
        }
        if (!tgt && !subs.empty()) tgt = &subs.front();
        if (tgt) taskJson["target_point"] = make_point_json_from_point(tgt->getPoint()); else taskJson["target_point"] = make_point_json(0,0,-1);
        // charge_parameters 在主流程里基于 AMR 信息填充
    } else if (typeStr == "STANDBY") {
        const SubTask* tgt = nullptr;
        for (const auto& st: subs){
            if (st.getPointType() == TaskFieldUtils::POINT_TYPE_STANDBY) { tgt=&st; break; }
        }
        if (!tgt && !subs.empty()) tgt = &subs.front();
        if (tgt) taskJson["target_point"] = make_point_json_from_point(tgt->getPoint()); else taskJson["target_point"] = make_point_json(0,0,-1);
        taskJson["standby_duration"] = 0;
        taskJson["interruptible"] = false;
    } else if (typeStr == "MAINTENANCE_CALL") {
        const SubTask* tgt = nullptr; if (!subs.empty()) tgt = &subs.front();
        if (tgt) taskJson["target_point"] = make_point_json_from_point(tgt->getPoint()); else taskJson["target_point"] = make_point_json(0,0,-1);
        taskJson["maintenance_type"] = "";
        taskJson["estimated_duration"] = 0;
    }
}

static bool amqp_ok(amqp_rpc_reply_t r, const char* ctx) {
    if (r.reply_type != AMQP_RESPONSE_NORMAL) {
        std::cerr << "AMQP error in " << ctx << ": reply_type=" << r.reply_type << std::endl;
        return false;
    }
    return true;
}

static bool env_enabled(const char* key) {
    const char* v = std::getenv(key);
    if (!v) return false;
    std::string s(v);
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
    return (s == "1" || s == "true" || s == "yes" || s == "on");
}

static bool result_pub_log_summary() {
    return env_enabled("RECEIVER_LOG_ALLOC_SUMMARY") || env_enabled("RESULT_PUBLISH_LOG_SUMMARY");
}

static bool result_pub_log_profile() {
    return env_enabled("RECEIVER_LOG_ALLOC_PROFILE") || env_enabled("RESULT_PUBLISH_LOG_PROFILE");
}

static bool result_pub_log_detail() {
    return env_enabled("RECEIVER_LOG_ALLOC_DETAIL") || env_enabled("RESULT_PUBLISH_LOG_DETAIL");
}

static std::string resolve_request_message_id(
    const SchedulingRequest* request,
    const std::vector<Task>& taskList
) {
    if (request && !request->messageId.empty()) return request->messageId;
    for (const auto& task : taskList) {
        if (!task.getMessageId().empty()) return task.getMessageId();
    }
    return std::string("REQ_") + iso8601_utc_now();
}

static std::string fallback_task_id(const Task& task, int idx) {
    if (!task.getMessageId().empty()) return task.getMessageId();
    return std::string("TASK_") + std::to_string(idx + 1);
}

static json make_current_position_json(const Amr& amr, const MapInfo* mapInfo) {
    int nodeId = amr.getCurrentNodeId();
    if (nodeId < 0) nodeId = amr.getNextDestinationPointId();
    if (nodeId < 0 && mapInfo) nodeId = mapInfo->findNearestNodeId(amr.getX(), amr.getY());
    json pos;
    pos["x"] = amr.getX();
    pos["y"] = amr.getY();
    pos["nodeId"] = node_id_to_string(nodeId);
    return pos;
}

static std::string ensure_timestamp(const std::string& iso, int fallbackSeconds) {
    if (!iso.empty()) return iso;
    if (fallbackSeconds <= 0) return iso8601_utc_now();
    return iso8601_utc_after_seconds(fallbackSeconds);
}

// 将分配结果封装为 JSON 对象（仅分配信息）
static json build_allocation_json(
    const AllocationResult& result,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    const MapInfo* mapInfo,
    long calculation_time_ms,
    double optimization_score,
    const std::vector<std::string>& constraints_violated,
    const SchedulingRequest* scheduling_request,
    int status_code
) {
    json root;
    root["messageId"] = resolve_request_message_id(scheduling_request, taskList);
    root["status"] = status_code;
    root["agv_assignments"] = json::array();
    std::vector<std::vector<int>> amrTasks = result.amrTasks.empty() ? TaskAllocationUtils::parseSolution(result.code) : result.amrTasks;
    if (amrTasks.size() < amrList.size()) amrTasks.resize(amrList.size());
    for (size_t ai = 0; ai < amrList.size(); ++ai) {
        const Amr& amr = amrList[ai];
        const auto& seq = amrTasks[ai];
        json agvEntry;
        agvEntry["agv_id"] = amr.getDeviceId();
        agvEntry["current_position"] = make_current_position_json(amr, mapInfo);
        json assigned = json::array();
        for (size_t k = 0; k < seq.size(); ++k) {
            int tIdx = seq[k];
            if (tIdx < 0 || tIdx >= static_cast<int>(taskList.size())) continue;
            const Task& task = taskList[tIdx];
            json taskJson;
            taskJson["task_id"] = fallback_task_id(task, tIdx);
            taskJson["task_sequence"] = static_cast<int>(k + 1);
            std::string typeStr = infer_task_type_str(task);
            if (typeStr.empty()) typeStr = "UNKNOWN";
            taskJson["task_type"] = typeStr;
            taskJson["priority"] = task.getPriority();
            taskJson["expected_start_time"] = ensure_timestamp(task.getExpectedStartTimeISO(), 0);
            taskJson["expected_completion_time"] = ensure_timestamp(task.getExpectedCompletionTimeISO(), 60);
            fill_task_type_specific(task, taskJson);
            if (typeStr == "CHARGE") {
                json charge;
                charge["min_battery_level"] = task.getMinBatteryLevel();
                charge["target_battery_level"] = static_cast<int>(std::llround(amr.getBatteryUpperLimitPct()));
                charge["max_charge_time"] = task.getExpectedCompletionTime();
                taskJson["charge_parameters"] = std::move(charge);
            }
            assigned.push_back(std::move(taskJson));
        }
        agvEntry["assigned_tasks"] = std::move(assigned);
        root["agv_assignments"].push_back(std::move(agvEntry));
    }

    // 额外信息：路径规划结果（按需填充）
    root["path_planning_results"] = json::array();

    // 未分配任务列表（按 taskId 字符串）
    json uaIds = json::array();
    for (int idx0 : result.unallocatedTaskIds) {
        if (idx0 >= 0 && idx0 < static_cast<int>(taskList.size())) {
            uaIds.push_back(fallback_task_id(taskList[idx0], idx0));
        }
    }
    root["unassigned_tasks"] = std::move(uaIds);

    json metadata;
    metadata["calculation_time"] = static_cast<int>(calculation_time_ms);
    metadata["optimization_score"] = optimization_score;
    json constraintArr = json::array();
    for (const auto& c : constraints_violated) constraintArr.push_back(c);
    metadata["constraints_violated"] = std::move(constraintArr);
    if (scheduling_request && !scheduling_request->allocationAlgorithm.empty()) {
        metadata["allocator"] = scheduling_request->allocationAlgorithm;
    }
    root["allocation_metadata"] = std::move(metadata);

    (void)mapInfo; // 地图信息在后续路径规划阶段使用
    return root;
}

// 全局缓存：静态表 + A*（按进程复用，避免重复加载/构建）
static StaticPathTable* g_cachedStaticTable = nullptr;
static const MapInfo* g_cachedTableMap = nullptr;
static std::string g_cachedStaticTablePath;
static AStarPathFinder* g_cachedAStar = nullptr;

static ShortestPathUpdater* g_cachedSpUpdater = nullptr;
static const MapInfo* g_cachedSpMap = nullptr;
static std::mutex g_spMutex;

static std::string describe_sp_state(double spVal) {
    if (spVal == static_cast<double>(DistanceState::UNCALCULATED)) return "UNCALCULATED";
    if (spVal == static_cast<double>(DistanceState::UNREACHABLE)) return "UNREACHABLE";
    if (!(spVal > 0.0)) return "INVALID";
    return std::to_string(spVal * 1000.0) + " ms";
}

static double probe_shortest_path_cost(const MapInfo* mapInfo, int startNode, int endNode, int amrType) {
    if (!mapInfo || startNode < 0 || endNode < 0) {
        return static_cast<double>(DistanceState::UNCALCULATED);
    }
    std::lock_guard<std::mutex> lk(g_spMutex);
    if (!g_cachedSpUpdater || g_cachedSpMap != mapInfo) {
        if (g_cachedSpUpdater) {
            g_cachedSpUpdater->requestExit();
            delete g_cachedSpUpdater;
            g_cachedSpUpdater = nullptr;
        }
        g_cachedSpUpdater = new ShortestPathUpdater(
            *mapInfo,
            PathPlanningConstants::kDefaultTurnPenaltyMm,
            1
        );
        // 初始化为(当前查询类型+1)维，后续查询可再次放大
        g_cachedSpUpdater->initializeMatrix(std::max(1, amrType + 1));
        g_cachedSpMap = mapInfo;
    }
    // 确保矩阵维度至少覆盖本次查询 amrType（可重复调用安全）
    g_cachedSpUpdater->initializeMatrix(std::max(1, amrType + 1));
    std::vector<int> targets{endNode};
    g_cachedSpUpdater->batchQueryByNodeId(startNode, targets, amrType);
    return g_cachedSpUpdater->queryByNodeId(startNode, endNode, amrType);
}

// 在已有分配 JSON 上补充路径规划信息（path_plan + metadata）
static std::vector<PathPlanningHelper::AmrPlanInfo> augment_with_path_planning(
    json& root,
    const AllocationResult& result,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    const MapInfo* mapInfo,
    bool attachToJson,
    const StaticPathTable* preloadedStatic = nullptr
) {
    if (!mapInfo) return {};

    // 1. Setup planner configuration based on debug mode
    GlobalPathPlanner::Config config;
    // 允许通过 SKIP_STATIC_TABLE 跳过静态表（与 external_receiver 行为一致）
    bool skipStatic = env_enabled("SKIP_STATIC_TABLE");
    bool isPureAStar = skipStatic || env_enabled("DEBUG_FORCE_PURE_ASTAR");

    if (!isPureAStar) {
        if (preloadedStatic) {
            config.staticTablePath.clear(); // 将通过预加载表
        } else {
            auto searchRoots = PathUtils::commonRoots();
            PathUtils::addRootIfSet(searchRoots, "AGV_SCHED_ROOT");
            PathUtils::addRootIfSet(searchRoots, "AGV_SCHED_ROOT", std::filesystem::path("config"));

            std::string staticTablePath = getenv_str("PLANNER_STATIC_TABLE", "config/south_20260107_all.bin");
            config.staticTablePath = PathUtils::resolvePathOrWarn(staticTablePath, "static path table", searchRoots);
        }
    } else {
        config.staticTablePath.clear(); // 不加载静态表
        config.enableMixedPlanning = false;
        if (result_pub_log_profile()) {
            std::cout << "ResultPublisher: SKIP_STATIC_TABLE=1，路径规划使用纯A*。" << std::endl;
        }
    }
    config.forcePureAStar = isPureAStar;
    config.verboseLogging = result_pub_log_profile() || result_pub_log_detail();

    auto make_planner = [&](const GlobalPathPlanner::Config& cfg,
                            const StaticPathTable* staticTable) -> std::unique_ptr<GlobalPathPlanner> {
        auto ptr = std::make_unique<GlobalPathPlanner>(*mapInfo, cfg);
        if (staticTable) {
            ptr->setStaticPathTable(staticTable);
        }
        if (!ptr->initialize()) {
            return nullptr;
        }
        return ptr;
    };

    // 2. Initialize the planner (fallback to pure A* when static table is invalid)
    std::unique_ptr<GlobalPathPlanner> plannerPtr = make_planner(config, preloadedStatic);
    if (!plannerPtr && !isPureAStar) {
        std::cerr << "ResultPublisher: GlobalPathPlanner failed to initialize (static table). "
                  << "Fallback to pure A*." << std::endl;
        GlobalPathPlanner::Config fallback = config;
        fallback.staticTablePath.clear();
        fallback.enableMixedPlanning = false;
        fallback.forcePureAStar = true;
        fallback.verboseLogging = true;
        plannerPtr = make_planner(fallback, nullptr);
    }
    if (!plannerPtr) {
        std::cerr << "ResultPublisher: GlobalPathPlanner failed to initialize." << std::endl;
        return {}; // Cannot proceed
    }
    GlobalPathPlanner& planner = *plannerPtr;

    std::vector<PathPlanningHelper::AmrPlanInfo> plans;
    bool anyPlanned = false;

    try {
        // 3. Build path plans using the centralized planner
        plans = PathPlanningHelper::BuildAmrPathPlans(result, amrList, taskList, planner, *mapInfo);

        std::unordered_map<std::string, const PathPlanningHelper::AmrPlanInfo*> planLookup;
        for (const auto& plan : plans) {
            planLookup[plan.amrId] = &plan;
        }

        // 4. Format the results into the JSON object
        json ppr = json::array();
        double globalSpeedMps = std::max(0.0, mapInfo->getGlobalMaxSpeed() / 1000.0);

        if (root.contains("agv_assignments") && root["agv_assignments"].is_array()) {
            for (auto& agvEntry : root["agv_assignments"]) {
                std::string amrId = agvEntry.value("agv_id", "");
                auto it = planLookup.find(amrId);
                if (it == planLookup.end()) continue;
                const auto* planInfo = it->second;

                std::vector<int> routeNodes;
                for (const auto& seg : planInfo->segments) {
                    if (seg.nodes.empty()) continue;
                    if (routeNodes.empty()) {
                        routeNodes.insert(routeNodes.end(), seg.nodes.begin(), seg.nodes.end());
                    } else if (!routeNodes.empty() && routeNodes.back() == seg.nodes.front()) {
                        routeNodes.insert(routeNodes.end(), seg.nodes.begin() + 1, seg.nodes.end());
                    } else {
                        routeNodes.insert(routeNodes.end(), seg.nodes.begin(), seg.nodes.end());
                    }
                }
                json complete_route;
                for (int nid : routeNodes) complete_route.push_back(std::to_string(nid));

                json segments = json::array();
                for (const auto& seg : planInfo->segments) {
                    json js;
                    js["from_node"] = std::to_string(seg.fromNodeId);
                    js["to_node"]   = std::to_string(seg.toNodeId);
                    js["estimated_travel_time"] = (int)std::llround(seg.timeSec);
                    js["max_speed"] = globalSpeedMps;
                    if (!seg.subTaskId.empty()) {
                        js["sub_task_id"] = seg.subTaskId;
                        js["sub_task_sequence"] = seg.subTaskSequence;
                    }
                    segments.push_back(js);
                }

                json timeline = json::array();
                int accSec = 0;
                for (const auto& seg : planInfo->segments) {
                    accSec += (int)std::llround(seg.timeSec + seg.serviceTimeSec);
                    json op;
                    op["node_id"] = std::to_string(seg.toNodeId);
                    op["operation_type"] = seg.stepType;
                    op["estimated_time"] = iso8601_utc_after_seconds(accSec);
                    if (!seg.subTaskId.empty()) {
                        op["sub_task_id"] = seg.subTaskId;
                        op["sub_task_sequence"] = seg.subTaskSequence;
                    }
                    timeline.push_back(op);
                }

                json one;
                one["agv_id"] = amrId;
                one["complete_route"] = std::move(complete_route);
                one["path_segments"] = std::move(segments);
                one["operation_timeline"] = std::move(timeline);
                one["total_path_length"] = (int)std::llround(planInfo->totalDistanceMm / 1000.0);
                one["total_travel_time"] = (int)std::llround(planInfo->totalTimeSec);
                ppr.push_back(std::move(one));
                
                agvEntry["estimated_start_time"] = iso8601_utc_now();
                agvEntry["estimated_completion_time"] = iso8601_utc_after_seconds((int)std::llround(planInfo->totalTimeSec));
                anyPlanned = anyPlanned || !planInfo->segments.empty();
            }
        }
        if (attachToJson) {
            root["path_planning_results"] = std::move(ppr);
        }

    } catch (const std::exception& e) {
        std::cerr << "ResultPublisher: error while generating paths: " << e.what() << std::endl;
        return {};
    }

    if (!anyPlanned) {
        std::cout << "ResultPublisher: no tasks assigned, nothing to plan.\n";
    }
    return plans;
}

// ResultPublisher 实现：持久化 RabbitMQ 连接
struct ResultPublisher::Impl {
    explicit Impl(const RabbitMQConfig& cfgIn)
        : cfg(cfgIn),
          conn(nullptr),
          connected(false),
          channel_open(false),
          message_ttl_ms(0) {
        const char* e = std::getenv("ASSIGN_RESULT_EXCHANGE");
        const char* q = std::getenv("ASSIGN_RESULT_QUEUE");
        const char* r = std::getenv("ASSIGN_RESULT_ROUTING_KEY");
        const char* b = std::getenv("ASSIGN_RESULT_BINDING_KEY");
        exch = e ? e : cfg.algoToDispExchange;
        queue = q ? q : cfg.algoToDispQueue;
        rkey = r ? r : cfg.algoToDispRoutingKey;
        binding_key = b ? b : cfg.algoToDispBindingKey;
        if (binding_key.empty()) binding_key = "#";
        if (const char* ttlEnv = std::getenv("ASSIGN_RESULT_TTL_MS")) {
            try {
                int parsed = std::stoi(ttlEnv);
                if (parsed >= 0) message_ttl_ms = parsed;
            } catch (...) {}
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
            std::cerr << "ResultPublisher: cannot create TCP socket" << std::endl;
            close();
            return false;
        }
        if (amqp_socket_open(sock, cfg.host.c_str(), cfg.port)) {
            std::cerr << "ResultPublisher: socket open failed" << std::endl;
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
        props.delivery_mode = 2; // 持久化
        if (message_ttl_ms > 0) {
            props._flags |= AMQP_BASIC_EXPIRATION_FLAG;
            props.expiration = amqp_cstring_bytes(expiration_str.c_str());
        }
        int rc = amqp_basic_publish(conn, 1,
                                    amqp_cstring_bytes(exch.c_str()),
                                    amqp_cstring_bytes(rkey.c_str()),
                                    0, 0, &props, message_bytes);
        if (rc != 0) {
            std::cerr << "ResultPublisher: publish failed rc=" << rc << std::endl;
            // 下次调用时尝试重连
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

ResultPublisher::ResultPublisher(const RabbitMQConfig& cfg)
    : impl_(new Impl(cfg)),
      dump_directory_(),
      dump_enabled_(false) {}

ResultPublisher::~ResultPublisher() {
    delete impl_;
    impl_ = nullptr;
}

void ResultPublisher::setDumpDirectory(const std::string& directory) {
    dump_directory_ = directory;
    dump_enabled_ = !dump_directory_.empty();
}

std::optional<std::vector<PathPlanningHelper::AmrPlanInfo>> ResultPublisher::publish(
    const AllocationResult& result,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    const MapInfo* mapInfo,
    long calculation_time_ms,
    double optimization_score,
    const std::vector<std::string>& constraints_violated,
    const SchedulingRequest* scheduling_request,
    const StaticPathTable* staticTable,
    int status_code
) {
    if (!impl_) return std::nullopt;
    const bool debug_dump_only = env_enabled("DEBUG_RESULT_DUMP_ONLY") || env_enabled("DEBUG_PATH_DUMP_ONLY");
    const bool attach_paths_in_allocation = env_enabled("PUBLISH_PATH_WITH_ALLOCATION")
        || env_enabled("PUBLISH_PATH_ON_ASSIGNMENT");
    const bool force_paths_in_payload = env_enabled("DEBUG_INCLUDE_PATH_RESULT");
    const bool include_path_responses = env_enabled("DEBUG_INCLUDE_PATH_RESPONSE");
    // 发布（MQ）是否携带路径：仅由明确的 publish 开关控制；调试落盘开关不应影响 MQ payload。
    const bool include_paths_in_publish = attach_paths_in_allocation;
    // 落盘（dump）是否携带路径：调试开关或显式 publish 开关触发（便于离线排障/可视化）。
    const bool include_paths_in_dump = force_paths_in_payload || debug_dump_only || attach_paths_in_allocation;
    // 1) 构造仅包含分配信息的 JSON
    json root = build_allocation_json(
        result, amrList, taskList, mapInfo,
        calculation_time_ms, optimization_score, constraints_violated,
        scheduling_request, status_code);

    // 2) 在同一 message 上补充路径规划结果（若提供地图信息）
    std::vector<PathPlanningHelper::AmrPlanInfo> plans;
    if (mapInfo) {
        plans = augment_with_path_planning(root, result, amrList, taskList, mapInfo, include_paths_in_dump, staticTable);
        if (!attach_paths_in_allocation) {
            // 若不随分配消息发布路径，仅用于缓存/后续 RobotPathRequest 响应
            if (result_pub_log_profile()) {
                std::cout << "ResultPublisher: 路径规划结果已计算并缓存，但未随分配结果发布。" << std::endl;
            }
        }
        if (include_path_responses) {
            std::string respMsgId;
            try {
                respMsgId = root.value("messageId", std::string());
            } catch (...) {
                respMsgId.clear();
            }
            root["path_response_by_agv"] = build_path_response_map(amrList, plans, *mapInfo, respMsgId);
        }
    }

    const std::string payload_for_dump = root.dump();
    const bool debug_log_result = env_enabled("DEBUG_LOG_RESULT") || result_pub_log_detail();
    if (debug_log_result) {
        std::cout << "ResultPublisher: dump JSON\n" << payload_for_dump << std::endl;
    }
    const bool dump_all_status = env_enabled("RESULT_DUMP_ALL_STATUS");
    const bool dump_only_on_success = !dump_all_status && !debug_dump_only;
    if (dump_enabled_ && (!dump_only_on_success || status_code == 0)) {
        dump_result_to_file(payload_for_dump, dump_directory_);
    }
    if (debug_dump_only) {
        if (result_pub_log_summary() || result_pub_log_profile()) {
            std::cout << "ResultPublisher: DEBUG_RESULT_DUMP_ONLY=1, skip RabbitMQ publish, only dump." << std::endl;
        }
        return plans;
    }

    // 默认 publish 不携带路径；若仅为了落盘附加了路径信息，则在 publish 前剔除相关字段。
    if (!include_paths_in_publish) {
        if (root.contains("path_planning_results")) {
            root.erase("path_planning_results");
        }
    }
    // Debug-only 字段默认不对外发布（避免影响业务消费者）。
    if (root.contains("path_response_by_agv")) {
        root.erase("path_response_by_agv");
    }
    const std::string payload_for_publish = root.dump();
    if (debug_log_result) {
        std::cout << "ResultPublisher: publish JSON\n" << payload_for_publish << std::endl;
    }
    if (impl_->publish(payload_for_publish)) {
        return plans;
    }
    return std::nullopt;
}

// 兼容旧接口：每次调用使用一次性连接
std::optional<std::vector<PathPlanningHelper::AmrPlanInfo>> PublishAllocationResult(
    const RabbitMQConfig& cfg,
    const AllocationResult& result,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    const MapInfo* mapInfo,
    long calculation_time_ms,
    double optimization_score,
    const std::vector<std::string>& constraints_violated,
    const SchedulingRequest* scheduling_request,
    const StaticPathTable* staticTable,
    int status_code
) {
    ResultPublisher publisher(cfg);
    return publisher.publish(result, amrList, taskList, mapInfo,
                             calculation_time_ms, optimization_score,
                             constraints_violated, scheduling_request, staticTable,
                             status_code);
}
// 简易UTC时间格式化（ISO8601, 秒精度）
static std::string iso8601_utc_now() {
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    std::time_t t = clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    return std::string(buf);
}

static std::string iso8601_utc_after_seconds(int seconds) {
    using clock = std::chrono::system_clock;
    auto now = clock::now() + std::chrono::seconds(seconds);
    std::time_t t = clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    return std::string(buf);
}

static std::string filename_timestamp_local() {
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    std::time_t t = clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    return std::string(buf);
}

static void dump_result_to_file(const std::string& payload,
                                const std::string& directory) {
    if (directory.empty()) return;
    namespace fs = std::filesystem;
    try {
        fs::path dirPath(directory);
        if (!fs::exists(dirPath)) {
            std::error_code ec;
            if (!fs::create_directories(dirPath, ec) && ec) {
                std::cerr << "ResultPublisher: failed to create directory "
                          << dirPath << " error=" << ec.message() << std::endl;
                return;
            }
        }
        std::string filename = "allocation_result_" + filename_timestamp_local() + ".json";
        fs::path filePath = dirPath / filename;
        std::ofstream ofs(filePath, std::ios::out | std::ios::binary);
        if (!ofs) {
            std::cerr << "ResultPublisher: cannot open file for result dump: "
                      << filePath << std::endl;
            return;
        }
        ofs << payload;
        ofs.close();
        std::cout << "ResultPublisher: result dumped to " << filePath.string()
                  << std::endl;

        // 仅保留最近 N 份结果，避免 debug 目录无限增长（<=0 表示不裁剪）
        int keepMax = getenv_int("RESULT_DUMP_KEEP_MAX", 0);
        if (keepMax > 0) {
            std::error_code ec;
            std::vector<std::pair<fs::file_time_type, fs::path>> dumps;
            for (const auto& entry : fs::directory_iterator(dirPath, ec)) {
                if (ec) break;
                if (!entry.is_regular_file(ec) || ec) { ec.clear(); continue; }
                fs::path p = entry.path();
                if (p.extension() != ".json") continue;
                std::string name = p.filename().string();
                if (name.rfind("allocation_result_", 0) != 0) continue;
                if (name == "allocation_result_latest.json") continue;
                fs::file_time_type ts = entry.last_write_time(ec);
                if (ec) { ec.clear(); continue; }
                dumps.emplace_back(ts, std::move(p));
            }
            std::sort(dumps.begin(), dumps.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });
            for (size_t i = static_cast<size_t>(keepMax); i < dumps.size(); ++i) {
                fs::remove(dumps[i].second, ec);
                if (ec) ec.clear();
            }
        }
    } catch (const std::exception& ex) {
        std::cerr << "ResultPublisher: failed to dump result JSON: "
                  << ex.what() << std::endl;
    }
}
