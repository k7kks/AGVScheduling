extern "C" {
#include <amqp.h>
#include <amqp_tcp_socket.h>
}
#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <chrono>
#include <fstream>
#include <set>
#include <unordered_set>
#include <thread>
#include <algorithm>
#include <unordered_map>
#include <optional>
#include <mutex>
#include <atomic>
#include <sstream>
#include <memory>
#include <iomanip>
#include <filesystem>
#include <condition_variable>
#include <queue>
#include <shared_mutex>
#include <functional>
#include <deque>

#include "common/RabbitMQConfig.h"
#include "common/CongestionRegionUtils.h"
#include "common/PathUtils.h"
#include "common/NodeReservationTable.h"
#include "nlohmann/json.hpp"

// Allocation pipeline includes
#include "data/MapInfo.h"
#include "parser/JsonParser.h"
#include "algorithm/ShortestPathUpdater.h"
#include "algorithm/CostMatrixGenerator.h"

static std::string getenv_str(const char* key, const char* defv);
static int getenv_int(const char* key, int defv);
static double getenv_double(const char* key, double defv);
static bool env_enabled(const char* key);
static bool env_enabled_default_true(const char* key);
static bool amqp_ok(amqp_rpc_reply_t r, const char* ctx);
#include "algorithm/GreedyTaskAllocator.h"
#include "algorithm/PostaTaskAllocator.h"
#if __has_include("algorithm/MlpTaskAllocator.h")
#include "algorithm/MlpTaskAllocator.h"
#define HAS_MLP_ALLOCATOR 1
#else
#define HAS_MLP_ALLOCATOR 0
#endif
#include "common/TaskPolicy.h"
#include "common/PathPlanningConstants.h"
#include "common/StatusArbitration.h"
#include "common/TaskReachabilityFilter.h"
#include "common/SchedulingRequest.h"
#include "common/TaskFieldUtils.h"
#include "algorithm/base/TaskAllocationUtils.h"
#include "data/Task.h"
#include "data/SubTask.h"
#include "message/ResultPublisher.h"
#include "message/AlgoPublisher.h"
#include "message/TrafficPathPublisher.h"
#include "StaticPathTable.h"
#include "path_planning/PathPlanningHelper.h"
#include "AStarPathFinder.h"
#include "common/AllocationReachability.h"
#include "common/AmrPositionResolver.h"

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;

namespace {

static long long current_millis();
static std::string log_time_prefix();

enum class TrailVersionMode {
    V1,
    V2,
    V100,
    UNKNOWN
};

static TrailVersionMode g_trailVersion = TrailVersionMode::UNKNOWN;

static std::string to_lower_ascii(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
}

static TrailVersionMode detect_trail_version_mode() {
    std::vector<std::filesystem::path> roots = PathUtils::commonRoots();
    PathUtils::addRootIfSet(roots, "AGV_SCHED_ROOT");
    for (const auto& root : roots) {
        std::string name = to_lower_ascii(root.filename().string());
        if (name.empty()) continue;
        if (name.find("v100") != std::string::npos) return TrailVersionMode::V100;
        if (name.find("v2") != std::string::npos) return TrailVersionMode::V2;
        if (name.find("v1") != std::string::npos) return TrailVersionMode::V1;
    }
    return TrailVersionMode::UNKNOWN;
}

static bool trail_periodic_enabled(TrailVersionMode mode) {
    if (mode == TrailVersionMode::V1) return false;
    // 默认关闭周期性 trail 推送（仅在显式配置间隔时启用）。
    return getenv_int("TRAIL_PUBLISH_INTERVAL_MS", 0) > 0;
}

static bool traffic_periodic_enabled(TrailVersionMode mode) {
    return mode != TrailVersionMode::V1;
}

static bool trail_send_on_allocation(TrailVersionMode mode) {
    return mode == TrailVersionMode::V1;
}

static const char* trail_version_label(TrailVersionMode mode) {
    switch (mode) {
        case TrailVersionMode::V1: return "V1";
        case TrailVersionMode::V2: return "V2";
        case TrailVersionMode::V100: return "V100";
        default: return "UNKNOWN";
    }
}

int parse_request_type(const json& root) {
    if (!root.contains("requestType")) return TaskFieldUtils::REQUEST_TYPE_UNKNOWN;
    const auto& v = root.at("requestType");
    if (v.is_number_integer()) {
        return v.get<int>();
    }
    if (v.is_string()) {
        return TaskFieldUtils::RequestTypeFromString(v.get<std::string>());
    }
    return TaskFieldUtils::REQUEST_TYPE_UNKNOWN;
}

int parse_batch_config(const json& root) {
    if (!root.contains("batchConfig")) return -1;
    const auto& bc = root.at("batchConfig");
    if (bc.is_number_integer()) {
        return bc.get<int>();
    }
    if (bc.is_object()) {
        return bc.value("maxBatchSize", -1);
    }
    return -1;
}

int parse_point_type(const json& subTask) {
    if (!subTask.contains("pointType")) return TaskFieldUtils::POINT_TYPE_UNKNOWN;
    const auto& pt = subTask.at("pointType");
    if (pt.is_number_integer()) {
        return pt.get<int>();
    }
    if (pt.is_string()) {
        return TaskFieldUtils::PointTypeFromString(pt.get<std::string>());
    }
    return TaskFieldUtils::POINT_TYPE_UNKNOWN;
}

struct RobotStatusEntry {
    std::string deviceId;
    int mapId = 0;
    std::string curArea;
    bool connection = false;
    double x = 0.0;
    double y = 0.0;
    double angle = 0.0;
    double speed = 0.0;
    std::string nodeId;
    json nextDestination;
    std::vector<json> trailPoints;
    std::string taskId;
    int deviceType = 0;
    double estimatedDurationSec = 0.0;
    int taskStatus = 0;
    int taskProgress = 0;
    int batteryLevel = 0;
    int endurance = 0;
    bool load = false;
    int errorCode = 0;
    int updateTime = 0;
};

struct RobotConfigEntry {
    std::string agvId;
    json rawConfig;
};

static bool status_is_idle(const RobotStatusEntry& entry) {
    // 0=空闲,2=任务完成(视作可分配); must also have empty taskId to be truly idle.
    return entry.taskId.empty() && (entry.taskStatus == 0 || entry.taskStatus == 2);
}

struct PlanCacheEntry {
    PathPlanningHelper::AmrPlanInfo plan;
    std::string schedulingRequestId;
    std::string generatedTime;
    long long pathId = 0;
    std::unordered_map<std::string, int> taskPriorities;
    std::chrono::steady_clock::time_point lastUpdate = std::chrono::steady_clock::now();
};

struct DynamicPlanCacheEntry {
    PathPlanningHelper::AmrPlanInfo plan;
    long long basePathId = 0;
    long long pathId = 0;
    int reserveBudget = 0;
    std::chrono::steady_clock::time_point lastUpdate = std::chrono::steady_clock::now();
};

struct TargetPointEntry {
    int nodeId = -1;
    std::string taskId;
    std::string subTaskId;
    int subTaskSequence = -1;
    std::string stepType;
};

struct PlanRuntimeState {
    long long planPathId = 0;
    std::deque<TargetPointEntry> targets;
    std::vector<int> reservedNodes;
    std::chrono::steady_clock::time_point blockedSince{};
    int tempGoalNodeId = -1;
    int lastFirstHopFromNodeId = -1;
    int lastFirstHopNodeId = -1;
    std::chrono::steady_clock::time_point lastFirstHopSince{};
    std::unordered_map<int, std::chrono::steady_clock::time_point> tempGoalBans;
    std::deque<int> tempGoalTabu;
    std::chrono::steady_clock::time_point tempGoalFreezeUntil{};
    int majorFailCount = 0;
    std::chrono::steady_clock::time_point lastMajorAttempt{};
    std::chrono::steady_clock::time_point stuckSince{};
    int stuckNodeId = -1;
    std::chrono::steady_clock::time_point staticSince{};
    int staticNodeId = -1;
    std::chrono::steady_clock::time_point lastStatusSeen{};
    double lastStatusX = 0.0;
    double lastStatusY = 0.0;
    double lastStatusAngle = 0.0;
    int lastStatusNodeId = -1;
    // If status nodeId is not inside the committed/reserved prefix, keep it for a short grace
    // period, then reset to the status node to avoid "phantom committed" blocks.
    std::chrono::steady_clock::time_point statusMismatchSince{};
    int statusMismatchNodeId = -1;
    bool waitTaskStatusClear = false;
    std::chrono::steady_clock::time_point waitTaskStatusReadyAt{};
};

enum class ReplanStage {
    NONE = -1,
    FAST = 0,
    MAJOR = 1,
    SUPER = 2,
};

static const char* replan_stage_label(ReplanStage stage) {
    switch (stage) {
        case ReplanStage::FAST: return "FAST";
        case ReplanStage::MAJOR: return "MAJOR";
        case ReplanStage::SUPER: return "SUPER";
        case ReplanStage::NONE: default: return "NONE";
    }
}

struct FallbackPathEntry {
    int nodeId = -1;
    long long pathId = 0;
    std::chrono::steady_clock::time_point lastUpdate = std::chrono::steady_clock::now();
};

struct TrailProgressInfo {
    size_t anchorIndex = 0;
    size_t routeCursor = 0;
    std::unordered_map<std::string, size_t> subTaskIndices;
    std::deque<std::string> subTaskOrder;
    std::vector<int> lastSentRoute;
};

struct TrafficPathRouteEntry {
    long long planPathId = 0;
    std::vector<int> nodes;
};

using StatusSource = StatusArbitration::StatusSource;

class RobotDataRepository {
public:
    void updateStatus(const RobotStatusEntry& entry, StatusSource source) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = statuses_.find(entry.deviceId);
        if (it != statuses_.end()) {
            StatusSource oldSource = StatusSource::UNKNOWN;
            auto sit = statusSources_.find(entry.deviceId);
            if (sit != statusSources_.end()) {
                oldSource = sit->second;
            }
            const int oldTs = it->second.updateTime;
            const int newTs = entry.updateTime;
            if (!StatusArbitration::ShouldAcceptStatusUpdate(oldSource, oldTs, source, newTs)) {
                return;
            }
        }
        statuses_[entry.deviceId] = entry;
        statusSources_[entry.deviceId] = source;
        statusSeen_[entry.deviceId] = std::chrono::steady_clock::now();
        auto parse_node_id_fast = [](const std::string& s) -> int {
            if (s.empty()) return -1;
            try {
                size_t idx = 0;
                int v = std::stoi(s, &idx);
                if (idx != s.size()) return -1;
                return v;
            } catch (...) {
                return -1;
            }
        };
        int nodeId = parse_node_id_fast(entry.nodeId);
        if (nodeId >= 0 && !entry.deviceId.empty()) {
            auto lit = lastStatusNode_.find(entry.deviceId);
            if (lit == lastStatusNode_.end()) {
                lastStatusNode_[entry.deviceId] = nodeId;
            } else if (lit->second != nodeId) {
                prevStatusNode_[entry.deviceId] = lit->second;
                lit->second = nodeId;
            }
        }
    }

    std::optional<int> getPrevStatusNodeId(const std::string& deviceId) const {
        if (deviceId.empty()) return std::nullopt;
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = prevStatusNode_.find(deviceId);
        if (it == prevStatusNode_.end()) return std::nullopt;
        return it->second;
    }

    void updateConfig(const RobotConfigEntry& entry) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = configs_.find(entry.agvId);
        if (it == configs_.end()) {
            configs_[entry.agvId] = entry;
            return;
        }
        RobotConfigEntry& stored = it->second;
        stored.agvId = entry.agvId;
        if (stored.rawConfig.is_object() && entry.rawConfig.is_object()) {
            stored.rawConfig.merge_patch(entry.rawConfig);
        } else {
            stored.rawConfig = entry.rawConfig;
        }
    }

    void updatePlan(const std::string& agvId,
                    const PathPlanningHelper::AmrPlanInfo& plan,
                    const std::string& schedId,
                    const std::string& genTime,
                    const std::unordered_map<std::string, int>& taskPriorities) {
        std::lock_guard<std::mutex> lk(mutex_);
        PlanCacheEntry entry;
        entry.plan = plan;
        entry.schedulingRequestId = schedId;
        entry.generatedTime = genTime;
        const long long newPathId = current_millis();
        entry.pathId = newPathId;
        entry.taskPriorities = taskPriorities;
        entry.lastUpdate = std::chrono::steady_clock::now();
        plans_[agvId] = std::move(entry);

        PlanRuntimeState rt;
        rt.planPathId = newPathId;
        auto existingRt = runtime_.find(agvId);
        if (existingRt != runtime_.end()) {
            // Preserve already-committed reserved window to avoid retracting control trail.
            rt.reservedNodes = existingRt->second.reservedNodes;
            rt.blockedSince = existingRt->second.blockedSince;
            rt.tempGoalNodeId = existingRt->second.tempGoalNodeId;
            rt.lastFirstHopFromNodeId = existingRt->second.lastFirstHopFromNodeId;
            rt.lastFirstHopNodeId = existingRt->second.lastFirstHopNodeId;
            rt.lastFirstHopSince = existingRt->second.lastFirstHopSince;
            rt.tempGoalBans = existingRt->second.tempGoalBans;
            rt.stuckSince = existingRt->second.stuckSince;
            rt.stuckNodeId = existingRt->second.stuckNodeId;
            rt.staticSince = existingRt->second.staticSince;
            rt.staticNodeId = existingRt->second.staticNodeId;
            rt.lastStatusSeen = existingRt->second.lastStatusSeen;
            rt.lastStatusX = existingRt->second.lastStatusX;
            rt.lastStatusY = existingRt->second.lastStatusY;
            rt.lastStatusAngle = existingRt->second.lastStatusAngle;
            rt.lastStatusNodeId = existingRt->second.lastStatusNodeId;
            rt.waitTaskStatusClear = existingRt->second.waitTaskStatusClear;
            rt.waitTaskStatusReadyAt = existingRt->second.waitTaskStatusReadyAt;
            // Preserve major fail history so SUPER 阶段的触发不会被每次分配刷新清零。
            rt.majorFailCount = existingRt->second.majorFailCount;
            rt.lastMajorAttempt = existingRt->second.lastMajorAttempt;
        }
        for (const auto& seg : plan.segments) {
            if (seg.toNodeId < 0) continue;
            TargetPointEntry tp;
            tp.nodeId = seg.toNodeId;
            tp.taskId = seg.taskId;
            tp.subTaskId = seg.subTaskId;
            tp.subTaskSequence = seg.subTaskSequence;
            tp.stepType = seg.stepType;
            rt.targets.push_back(std::move(tp));
        }
        runtime_[agvId] = std::move(rt);

        trailProgress_.erase(agvId);
        fallbackPaths_.erase(agvId);
        dynamicPlans_.erase(agvId);
        trafficPathRoutes_.erase(agvId);
    }

    std::optional<PlanCacheEntry> getPlan(const std::string& agvId) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = plans_.find(agvId);
        if (it == plans_.end()) return std::nullopt;
        return it->second;
    }

    bool getDynamicPlanIfFresh(const std::string& deviceId,
                               long long basePathId,
                               int reserveBudget,
                               int maxAgeMs,
                               PathPlanningHelper::AmrPlanInfo& out) const {
        if (deviceId.empty() || maxAgeMs <= 0) return false;
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = dynamicPlans_.find(deviceId);
        if (it == dynamicPlans_.end()) return false;
        if (basePathId > 0 && it->second.basePathId != basePathId) return false;
        if (it->second.reserveBudget != reserveBudget) return false;
        auto ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - it->second.lastUpdate).count();
        if (ageMs >= maxAgeMs) return false;
        out = it->second.plan;
        return true;
    }

    bool getDynamicPlanLatest(const std::string& deviceId,
                              long long basePathId,
                              int reserveBudget,
                              PathPlanningHelper::AmrPlanInfo& out) const {
        if (deviceId.empty()) return false;
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = dynamicPlans_.find(deviceId);
        if (it == dynamicPlans_.end()) return false;
        if (basePathId > 0 && it->second.basePathId != basePathId) return false;
        if (reserveBudget > 0 && it->second.reserveBudget != reserveBudget) return false;
        out = it->second.plan;
        return true;
    }

    bool getDynamicPlanEntry(const std::string& deviceId,
                             long long basePathId,
                             int reserveBudget,
                             DynamicPlanCacheEntry& out) const {
        if (deviceId.empty()) return false;
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = dynamicPlans_.find(deviceId);
        if (it == dynamicPlans_.end()) return false;
        if (basePathId > 0 && it->second.basePathId != basePathId) return false;
        if (reserveBudget > 0 && it->second.reserveBudget != reserveBudget) return false;
        out = it->second;
        return true;
    }

    void updateDynamicPlan(const std::string& deviceId,
                           const PathPlanningHelper::AmrPlanInfo& plan,
                           long long basePathId,
                           int reserveBudget,
                           bool refreshTimestamp = true) {
        if (deviceId.empty()) return;
        std::lock_guard<std::mutex> lk(mutex_);
        auto same_plan_nodes = [](const PathPlanningHelper::AmrPlanInfo& a,
                                  const PathPlanningHelper::AmrPlanInfo& b) -> bool {
            if (a.startNodeId != b.startNodeId) return false;
            if (a.segments.size() != b.segments.size()) return false;
            for (size_t i = 0; i < a.segments.size(); ++i) {
                const auto& sa = a.segments[i];
                const auto& sb = b.segments[i];
                if (sa.nodes != sb.nodes) return false;
                if (sa.nodes.empty()) {
                    if (sa.fromNodeId != sb.fromNodeId || sa.toNodeId != sb.toNodeId) return false;
                }
            }
            return true;
        };
        const auto now = std::chrono::steady_clock::now();
        const long long nowId = current_millis();
        long long newPathId = nowId > 0 ? nowId : 1;
        DynamicPlanCacheEntry entry;
        entry.plan = plan;
        entry.basePathId = basePathId;
        entry.reserveBudget = reserveBudget;
        entry.pathId = newPathId;
        auto it = dynamicPlans_.find(deviceId);
        if (it != dynamicPlans_.end() &&
            it->second.basePathId == basePathId &&
            it->second.reserveBudget == reserveBudget) {
            if (same_plan_nodes(it->second.plan, plan)) {
                entry.pathId = it->second.pathId;
            } else if (it->second.pathId > 0 && entry.pathId <= it->second.pathId) {
                entry.pathId = it->second.pathId + 1;
            }
        }
        if (!refreshTimestamp) {
            if (it != dynamicPlans_.end() &&
                it->second.basePathId == basePathId &&
                it->second.reserveBudget == reserveBudget) {
                entry.lastUpdate = it->second.lastUpdate;
            } else {
                entry.lastUpdate = now;
            }
        } else {
            entry.lastUpdate = now;
        }
        dynamicPlans_[deviceId] = std::move(entry);
    }

    void clearDynamicPlan(const std::string& deviceId) {
        if (deviceId.empty()) return;
        std::lock_guard<std::mutex> lk(mutex_);
        dynamicPlans_.erase(deviceId);
    }

    std::vector<std::pair<std::string, PlanCacheEntry>> snapshotPlans() {
        std::lock_guard<std::mutex> lk(mutex_);
        std::vector<std::pair<std::string, PlanCacheEntry>> out;
        out.reserve(plans_.size());
        for (const auto& kv : plans_) {
            out.emplace_back(kv.first, kv.second);
        }
        return out;
    }

    std::optional<TargetPointEntry> advanceTargets(const std::string& deviceId, int currentNodeId) {
        if (deviceId.empty() || currentNodeId < 0) return std::nullopt;
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = runtime_.find(deviceId);
        if (it == runtime_.end()) return std::nullopt;
        PlanRuntimeState& rt = it->second;
        bool progressed = false;
        while (!rt.targets.empty() && rt.targets.front().nodeId == currentNodeId) {
            rt.targets.pop_front();
            progressed = true;
        }
        if (progressed) {
            rt.blockedSince = std::chrono::steady_clock::time_point{};
            rt.tempGoalNodeId = -1;
            rt.lastFirstHopFromNodeId = -1;
            rt.lastFirstHopNodeId = -1;
            rt.lastFirstHopSince = std::chrono::steady_clock::time_point{};
            rt.tempGoalBans.clear();
            rt.majorFailCount = 0;
            rt.lastMajorAttempt = std::chrono::steady_clock::time_point{};
            rt.stuckSince = std::chrono::steady_clock::time_point{};
            rt.stuckNodeId = -1;
            rt.staticSince = std::chrono::steady_clock::time_point{};
            rt.staticNodeId = -1;
            rt.waitTaskStatusClear = true;
            rt.waitTaskStatusReadyAt = std::chrono::steady_clock::time_point{};
        }
        if (rt.targets.empty()) return std::nullopt;
        return rt.targets.front();
    }

    std::optional<TargetPointEntry> peekTarget(const std::string& deviceId) const {
        if (deviceId.empty()) return std::nullopt;
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = runtime_.find(deviceId);
        if (it == runtime_.end() || it->second.targets.empty()) return std::nullopt;
        return it->second.targets.front();
    }

    PlanRuntimeState snapshotRuntimeState(const std::string& deviceId) const {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = runtime_.find(deviceId);
        if (it == runtime_.end()) return PlanRuntimeState{};
        return it->second;
    }

	    void updateRuntimeState(const std::string& deviceId,
	                            long long expectedPlanPathId,
	                            const PlanRuntimeState& state) {
	        if (deviceId.empty()) return;
	        std::lock_guard<std::mutex> lk(mutex_);
	        auto it = runtime_.find(deviceId);
	        if (it == runtime_.end()) {
	            runtime_[deviceId] = state;
	            return;
	        }
	        PlanRuntimeState& stored = it->second;
	        if (expectedPlanPathId > 0 && stored.planPathId != expectedPlanPathId) {
	            return;
	        }
        stored.reservedNodes = state.reservedNodes;
        stored.blockedSince = state.blockedSince;
        stored.tempGoalNodeId = state.tempGoalNodeId;
        stored.lastFirstHopFromNodeId = state.lastFirstHopFromNodeId;
        stored.lastFirstHopNodeId = state.lastFirstHopNodeId;
        stored.lastFirstHopSince = state.lastFirstHopSince;
        stored.tempGoalBans = state.tempGoalBans;
        stored.tempGoalTabu = state.tempGoalTabu;
        stored.tempGoalFreezeUntil = state.tempGoalFreezeUntil;
        stored.majorFailCount = state.majorFailCount;
        stored.lastMajorAttempt = state.lastMajorAttempt;
        stored.stuckSince = state.stuckSince;
        stored.stuckNodeId = state.stuckNodeId;
        stored.staticSince = state.staticSince;
        stored.staticNodeId = state.staticNodeId;
        stored.lastStatusSeen = state.lastStatusSeen;
        stored.lastStatusX = state.lastStatusX;
        stored.lastStatusY = state.lastStatusY;
        stored.lastStatusAngle = state.lastStatusAngle;
        stored.lastStatusNodeId = state.lastStatusNodeId;
        stored.statusMismatchSince = state.statusMismatchSince;
        stored.statusMismatchNodeId = state.statusMismatchNodeId;
        stored.waitTaskStatusClear = state.waitTaskStatusClear;
        stored.waitTaskStatusReadyAt = state.waitTaskStatusReadyAt;
	        stored.planPathId = state.planPathId;
	        stored.targets = state.targets;
	    }

        // Status stream trimming should not depend on planPathId; it is about "where the robot is".
        void updateReservedNodesAndMismatchOnly(const std::string& deviceId,
                                               const std::vector<int>& reservedNodes,
                                               std::chrono::steady_clock::time_point mismatchSince,
                                               int mismatchNodeId) {
            if (deviceId.empty()) return;
            std::lock_guard<std::mutex> lk(mutex_);
            auto it = runtime_.find(deviceId);
            if (it == runtime_.end()) return;
            PlanRuntimeState& stored = it->second;
            stored.reservedNodes = reservedNodes;
            stored.statusMismatchSince = mismatchSince;
            stored.statusMismatchNodeId = mismatchNodeId;
        }

	    void updateReservedNodesOnly(const std::string& deviceId,
	                                 long long expectedPlanPathId,
	                                 const std::vector<int>& reservedNodes) {
	        if (deviceId.empty()) return;
	        std::lock_guard<std::mutex> lk(mutex_);
	        auto it = runtime_.find(deviceId);
	        if (it == runtime_.end()) return;
	        PlanRuntimeState& stored = it->second;
	        if (expectedPlanPathId > 0 && stored.planPathId != expectedPlanPathId) {
	            return;
	        }
	        stored.reservedNodes = reservedNodes;
	    }

    void updateAnchorProgress(const std::string& deviceId, size_t idx) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto& prog = trailProgress_[deviceId];
        if (idx > prog.anchorIndex) prog.anchorIndex = idx;
    }

    size_t getAnchorProgress(const std::string& deviceId) const {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = trailProgress_.find(deviceId);
        if (it == trailProgress_.end()) return 0;
        return it->second.anchorIndex;
    }

    void updateSubTaskProgress(const std::string& deviceId,
                               const std::string& subTaskId,
                               size_t idx) {
        if (subTaskId.empty()) return;
        std::lock_guard<std::mutex> lk(mutex_);
        auto& prog = trailProgress_[deviceId];
        auto& current = prog.subTaskIndices[subTaskId];
        if (idx > current) current = idx;
        prog.subTaskOrder.push_back(subTaskId);
        enforceSubTaskLimit(prog, subTaskId);
    }

    size_t getSubTaskProgress(const std::string& deviceId,
                              const std::string& subTaskId) const {
        if (subTaskId.empty()) return 0;
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = trailProgress_.find(deviceId);
        if (it == trailProgress_.end()) return 0;
        auto sit = it->second.subTaskIndices.find(subTaskId);
        if (sit == it->second.subTaskIndices.end()) return 0;
        return sit->second;
    }

    size_t getRouteCursor(const std::string& deviceId) const {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = trailProgress_.find(deviceId);
        if (it == trailProgress_.end()) return 0;
        return it->second.routeCursor;
    }

    void updateRouteCursor(const std::string& deviceId,
                           size_t idx,
                           size_t routeSize) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto& prog = trailProgress_[deviceId];
        size_t normalized = (routeSize == 0) ? 0 : std::min(idx, routeSize);
        if (routeSize > 0 && normalized >= routeSize) {
            normalized = 0;
        }
        prog.routeCursor = normalized;
        if (prog.anchorIndex > prog.routeCursor) {
            prog.anchorIndex = prog.routeCursor;
        }
        if (prog.routeCursor == 0) {
            prog.subTaskIndices.clear();
            prog.subTaskOrder.clear();
        }
    }

    std::vector<int> getLastSentRoute(const std::string& deviceId) const {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = trailProgress_.find(deviceId);
        if (it == trailProgress_.end()) return {};
        return it->second.lastSentRoute;
    }

    void updateLastSentRoute(const std::string& deviceId, const std::vector<int>& route) {
        std::lock_guard<std::mutex> lk(mutex_);
        trailProgress_[deviceId].lastSentRoute = route;
    }

    std::vector<int> updateTrafficGrowingRoute(const std::string& deviceId,
                                               long long planPathId,
                                               const std::string& routeKey,
                                               const std::vector<int>& latestRoute,
                                               int currentNodeId) {
        if (deviceId.empty() || routeKey.empty()) return latestRoute;
        std::lock_guard<std::mutex> lk(mutex_);
        auto& routeMap = trafficPathRoutes_[deviceId];
        auto& entry = routeMap[routeKey];

        auto merge_latest = [&](const std::vector<int>& history,
                                const std::vector<int>& latest) {
            if (latest.empty()) return history;
            if (history.empty()) return latest;

            const size_t maxOverlap = std::min(history.size(), latest.size());
            for (size_t overlap = maxOverlap; overlap > 0; --overlap) {
                auto suffixBegin = history.begin() + static_cast<std::ptrdiff_t>(history.size() - overlap);
                if (std::equal(suffixBegin, history.end(), latest.begin())) {
                    std::vector<int> merged = history;
                    merged.insert(merged.end(),
                                  latest.begin() + static_cast<std::ptrdiff_t>(overlap),
                                  latest.end());
                    return merged;
                }
            }

            if (currentNodeId >= 0) {
                auto histCur = std::find(history.begin(), history.end(), currentNodeId);
                auto latestCur = std::find(latest.begin(), latest.end(), currentNodeId);
                if (histCur != history.end() && latestCur != latest.end()) {
                    std::vector<int> merged(history.begin(), histCur + 1);
                    merged.insert(merged.end(), latestCur + 1, latest.end());
                    return merged;
                }
            }

            auto histStart = std::find(history.begin(), history.end(), latest.front());
            if (histStart != history.end()) {
                std::vector<int> merged(history.begin(), histStart + 1);
                if (latest.size() > 1) {
                    merged.insert(merged.end(), latest.begin() + 1, latest.end());
                }
                return merged;
            }

            std::vector<int> merged = history;
            if (!merged.empty() && merged.back() == latest.front()) {
                if (latest.size() > 1) {
                    merged.insert(merged.end(), latest.begin() + 1, latest.end());
                }
            } else {
                merged.insert(merged.end(), latest.begin(), latest.end());
            }
            return merged;
        };

        if (entry.planPathId != planPathId || entry.nodes.empty()) {
            entry.planPathId = planPathId;
            entry.nodes = latestRoute;
        } else {
            entry.nodes = merge_latest(entry.nodes, latestRoute);
        }
        entry.nodes.erase(std::unique(entry.nodes.begin(), entry.nodes.end()), entry.nodes.end());
        return entry.nodes;
    }

    std::vector<RobotStatusEntry> snapshotStatuses() const {
        std::lock_guard<std::mutex> lk(mutex_);
        std::vector<RobotStatusEntry> out;
        out.reserve(statuses_.size());
        for (const auto& kv : statuses_) {
            out.push_back(kv.second);
        }
        return out;
    }

    std::vector<std::string> collectStaleDevices(int staleMs,
                                                 std::chrono::steady_clock::time_point now) const {
        std::vector<std::string> out;
        if (staleMs <= 0) return out;
        std::lock_guard<std::mutex> lk(mutex_);
        const auto staleWindow = std::chrono::milliseconds(staleMs);
        for (const auto& kv : statusSeen_) {
            if (now - kv.second >= staleWindow) {
                out.push_back(kv.first);
            }
        }
        return out;
    }

    std::optional<RobotStatusEntry> getStatusById(const std::string& deviceId) const {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = statuses_.find(deviceId);
        if (it == statuses_.end()) return std::nullopt;
        return it->second;
    }

    long long ensureFallbackPathId(const std::string& deviceId,
                                   int nodeId,
                                   long long newPathId) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto& entry = fallbackPaths_[deviceId];
        if (entry.pathId > 0 && entry.nodeId == nodeId) {
            return entry.pathId;
        }
        entry.nodeId = nodeId;
        entry.pathId = newPathId;
        entry.lastUpdate = std::chrono::steady_clock::now();
        return entry.pathId;
    }

    void clearPlans() {
        std::lock_guard<std::mutex> lk(mutex_);
        plans_.clear();
        runtime_.clear();
        trailProgress_.clear();
        fallbackPaths_.clear();
        dynamicPlans_.clear();
        trafficPathRoutes_.clear();
    }

    void clearDynamicStateKeepPlans() {
        std::lock_guard<std::mutex> lk(mutex_);
        dynamicPlans_.clear();
        trailProgress_.clear();
        fallbackPaths_.clear();
        trafficPathRoutes_.clear();
        for (auto& kv : runtime_) {
            PlanRuntimeState& rt = kv.second;
            rt.reservedNodes.clear();
            rt.blockedSince = std::chrono::steady_clock::time_point{};
            rt.tempGoalNodeId = -1;
            rt.lastFirstHopFromNodeId = -1;
            rt.lastFirstHopNodeId = -1;
            rt.lastFirstHopSince = std::chrono::steady_clock::time_point{};
            rt.tempGoalBans.clear();
            rt.tempGoalTabu.clear();
            rt.tempGoalFreezeUntil = std::chrono::steady_clock::time_point{};
            rt.stuckSince = std::chrono::steady_clock::time_point{};
            rt.stuckNodeId = -1;
            rt.staticSince = std::chrono::steady_clock::time_point{};
            rt.staticNodeId = -1;
            rt.statusMismatchSince = std::chrono::steady_clock::time_point{};
            rt.statusMismatchNodeId = -1;
            rt.waitTaskStatusClear = false;
            rt.waitTaskStatusReadyAt = std::chrono::steady_clock::time_point{};
        }
    }

    void removeDevice(const std::string& deviceId) {
        if (deviceId.empty()) return;
        std::lock_guard<std::mutex> lk(mutex_);
        statuses_.erase(deviceId);
        statusSources_.erase(deviceId);
        lastStatusNode_.erase(deviceId);
        prevStatusNode_.erase(deviceId);
        statusSeen_.erase(deviceId);
        configs_.erase(deviceId);
        plans_.erase(deviceId);
        dynamicPlans_.erase(deviceId);
        runtime_.erase(deviceId);
        trailProgress_.erase(deviceId);
        fallbackPaths_.erase(deviceId);
        trafficPathRoutes_.erase(deviceId);
    }

	private:
	    mutable std::mutex mutex_;
	    std::unordered_map<std::string, RobotStatusEntry> statuses_;
        std::unordered_map<std::string, StatusSource> statusSources_;
        std::unordered_map<std::string, int> lastStatusNode_;
        std::unordered_map<std::string, int> prevStatusNode_;
        std::unordered_map<std::string, std::chrono::steady_clock::time_point> statusSeen_;
	    std::unordered_map<std::string, RobotConfigEntry> configs_;
	    std::unordered_map<std::string, PlanCacheEntry> plans_;
        std::unordered_map<std::string, DynamicPlanCacheEntry> dynamicPlans_;
        std::unordered_map<std::string, PlanRuntimeState> runtime_;
	    std::unordered_map<std::string, TrailProgressInfo> trailProgress_;
	    std::unordered_map<std::string, FallbackPathEntry> fallbackPaths_;
        std::unordered_map<std::string,
                           std::unordered_map<std::string, TrafficPathRouteEntry>> trafficPathRoutes_;

    static constexpr size_t kMaxTrackedSubTasks = 64;

    void enforceSubTaskLimit(TrailProgressInfo& prog,
                             const std::string& protectedKey) {
        if (kMaxTrackedSubTasks == 0) return;
        while (prog.subTaskIndices.size() > kMaxTrackedSubTasks &&
               !prog.subTaskOrder.empty()) {
            const std::string candidate = prog.subTaskOrder.front();
            prog.subTaskOrder.pop_front();
            if (candidate == protectedKey) continue;
            auto it = prog.subTaskIndices.find(candidate);
            if (it != prog.subTaskIndices.end()) {
                prog.subTaskIndices.erase(it);
                break;
            }
        }
        const size_t maxOrderSize = kMaxTrackedSubTasks * 8;
        if (maxOrderSize > 0 && prog.subTaskOrder.size() > maxOrderSize) {
            while (prog.subTaskOrder.size() > maxOrderSize) {
                prog.subTaskOrder.pop_front();
            }
        }
    }
};

class SchedulingTaskQueue {
public:
    using Job = std::function<void()>;

    SchedulingTaskQueue() : stop_(false) {}

    ~SchedulingTaskQueue() {
        stop();
    }

    void start(size_t workers) {
        if (workers == 0) workers = 1;
        std::lock_guard<std::mutex> lk(mutex_);
        if (!workers_.empty()) return;
        stop_ = false;
        for (size_t i = 0; i < workers; ++i) {
            workers_.emplace_back([this, i]() { this->runWorker(i); });
        }
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (stop_) return;
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& th : workers_) {
            if (th.joinable()) th.join();
        }
        workers_.clear();
    }

    void submit(Job job) {
        {
            std::lock_guard<std::mutex> lk(mutex_);
            jobs_.push(std::move(job));
        }
        cv_.notify_one();
    }

private:
    void runWorker(size_t /*index*/) {
        while (true) {
            Job job;
            {
                std::unique_lock<std::mutex> lk(mutex_);
                cv_.wait(lk, [&]() { return stop_ || !jobs_.empty(); });
                if (stop_ && jobs_.empty()) return;
                job = std::move(jobs_.front());
                jobs_.pop();
            }
            try {
                job();
            } catch (const std::exception& ex) {
                std::cerr << "[SchedulingWorker] exception: " << ex.what() << std::endl;
            } catch (...) {
                std::cerr << "[SchedulingWorker] unknown exception" << std::endl;
            }
        }
    }

    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<Job> jobs_;
    bool stop_;
    std::vector<std::thread> workers_;
};

class SchedulingBusyGuard {
public:
    explicit SchedulingBusyGuard(std::atomic<bool>& flag) : flag_(flag), active_(true) {}
    ~SchedulingBusyGuard() {
        if (active_) flag_.store(false);
    }
    SchedulingBusyGuard(const SchedulingBusyGuard&) = delete;
    SchedulingBusyGuard& operator=(const SchedulingBusyGuard&) = delete;
private:
    std::atomic<bool>& flag_;
    bool active_;
};

class SchedulingBusyCountGuard {
public:
    explicit SchedulingBusyCountGuard(std::atomic<int>& counter)
        : counter_(counter), active_(true) {
        counter_.fetch_add(1);
    }
    ~SchedulingBusyCountGuard() {
        if (active_) counter_.fetch_sub(1);
    }
    SchedulingBusyCountGuard(const SchedulingBusyCountGuard&) = delete;
    SchedulingBusyCountGuard& operator=(const SchedulingBusyCountGuard&) = delete;
private:
    std::atomic<int>& counter_;
    bool active_;
};

static std::string read_string(const json& obj, const char* key, const std::string& def);
static void collect_vehicle_filters(const json& taskObj, std::vector<std::string>& out);
static bool fill_status_entry_from_json(const json& item, RobotStatusEntry& entry);
static bool handle_map_info(const json& j,
                            MapInfo& mapInfo,
                            NodeReservationTable& reservations,
                            std::unique_ptr<AStarPathFinder>& aStarPtr,
                            const std::string& cachePath,
                            std::atomic<int>& mapId);
static double reserve_near_conflict_threshold_mm();

struct MapRuntimeContext {
    explicit MapRuntimeContext(int initialMapId,
                               bool defaultSkipStaticTable,
                               const std::string& defaultStaticTablePath)
        : mapId(initialMapId),
          mapReady(false),
          mapEpoch(0),
          activeMapId(initialMapId),
          mapInfoPtr(new MapInfo(2)),
          aStarPtr(new AStarPathFinder(*mapInfoPtr)),
          skipStaticTable(defaultSkipStaticTable),
          staticTableReady(false),
          staticTablePath(defaultStaticTablePath),
          schedulingBusy(false) {
        if (!skipStaticTable && !staticTablePath.empty()) {
            if (staticTable.loadFromFile(staticTablePath)) {
                staticTableReady = true;
            } else {
                skipStaticTable = true;
                staticTableReady = false;
            }
        }
    }

    int mapId = 0;
    std::string mapVersion;
    std::shared_mutex mapMutex;
    std::atomic<bool> mapReady;
    std::atomic<long long> mapEpoch;
    std::atomic<int> activeMapId;
    std::unique_ptr<MapInfo> mapInfoPtr;
    std::unique_ptr<AStarPathFinder> aStarPtr;
    NodeReservationTable nodeReservations;
    RobotDataRepository robotRepo;
    bool skipStaticTable = true;
    bool staticTableReady = false;
    StaticPathTable staticTable;
    std::string staticTablePath;
    std::mutex schedulingLatestMutex;
    std::optional<json> schedulingLatest;
    bool schedulingWorkerActive = false;
    std::atomic<bool> schedulingBusy;
};

class MultiMapManager {
public:
    MultiMapManager(bool defaultSkipStaticTable,
                    const std::string& defaultStaticTablePath)
        : defaultSkipStaticTable_(defaultSkipStaticTable),
          defaultStaticTablePath_(defaultStaticTablePath) {}

    std::shared_ptr<MapRuntimeContext> getOrCreateContext(int mapId) {
        std::lock_guard<std::mutex> lk(mutex_);
        int key = mapId > 0 ? mapId : 0;
        auto it = contexts_.find(key);
        if (it != contexts_.end()) return it->second;
        auto ctx = std::make_shared<MapRuntimeContext>(key, defaultSkipStaticTable_, defaultStaticTablePath_);
        contexts_[key] = ctx;
        return ctx;
    }

    std::vector<std::shared_ptr<MapRuntimeContext>> snapshotContexts() const {
        std::lock_guard<std::mutex> lk(mutex_);
        std::vector<std::shared_ptr<MapRuntimeContext>> out;
        out.reserve(contexts_.size());
        for (const auto& kv : contexts_) {
            out.push_back(kv.second);
        }
        std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
            return a->activeMapId.load() < b->activeMapId.load();
        });
        return out;
    }

    bool anyMapReady() const {
        auto contexts = snapshotContexts();
        for (const auto& ctx : contexts) {
            if (ctx->mapReady.load()) return true;
        }
        return false;
    }

    size_t readyContextCount() const {
        size_t count = 0;
        auto contexts = snapshotContexts();
        for (const auto& ctx : contexts) {
            if (ctx->mapReady.load()) count += 1;
        }
        return count;
    }

    std::shared_ptr<MapRuntimeContext> singleReadyContext() const {
        std::shared_ptr<MapRuntimeContext> out;
        auto contexts = snapshotContexts();
        for (const auto& ctx : contexts) {
            if (!ctx->mapReady.load()) continue;
            if (out) return nullptr;
            out = ctx;
        }
        return out;
    }

    void updateDeviceMap(const std::string& deviceId, int mapId) {
        if (deviceId.empty() || mapId <= 0) return;
        std::lock_guard<std::mutex> lk(mutex_);
        deviceToMap_[deviceId] = mapId;
    }

    std::optional<int> getDeviceMap(const std::string& deviceId) const {
        if (deviceId.empty()) return std::nullopt;
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = deviceToMap_.find(deviceId);
        if (it == deviceToMap_.end()) return std::nullopt;
        return it->second;
    }

    std::shared_ptr<MapRuntimeContext> resolveContextForDevice(const std::string& deviceId,
                                                               int requestedMapId = 0) {
        if (requestedMapId > 0) {
            auto ctx = getOrCreateContext(requestedMapId);
            if (deviceId.empty()) return ctx;
            auto mapped = getDeviceMap(deviceId);
            if (!mapped.has_value() || *mapped == requestedMapId) {
                return ctx;
            }
            return nullptr;
        }
        auto mapped = getDeviceMap(deviceId);
        if (mapped.has_value()) {
            return getOrCreateContext(*mapped);
        }
        return singleReadyContext();
    }

    void moveDeviceToContext(const std::string& deviceId, int newMapId) {
        if (deviceId.empty() || newMapId <= 0) return;
        std::shared_ptr<MapRuntimeContext> oldCtx;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            auto it = deviceToMap_.find(deviceId);
            if (it != deviceToMap_.end() && it->second == newMapId) return;
            if (it != deviceToMap_.end()) {
                auto oldIt = contexts_.find(it->second);
                if (oldIt != contexts_.end()) oldCtx = oldIt->second;
            }
            deviceToMap_[deviceId] = newMapId;
        }
        if (oldCtx) {
            oldCtx->nodeReservations.releaseAllByOwner(deviceId);
            oldCtx->robotRepo.removeDevice(deviceId);
        }
    }

    void updateConfig(const RobotConfigEntry& entry) {
        if (entry.agvId.empty()) return;
        std::lock_guard<std::mutex> lk(mutex_);
        configs_[entry.agvId] = entry;
    }

    std::optional<RobotConfigEntry> getConfig(const std::string& deviceId) const {
        if (deviceId.empty()) return std::nullopt;
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = configs_.find(deviceId);
        if (it == configs_.end()) return std::nullopt;
        return it->second;
    }

private:
    mutable std::mutex mutex_;
    std::unordered_map<int, std::shared_ptr<MapRuntimeContext>> contexts_;
    std::unordered_map<std::string, int> deviceToMap_;
    std::unordered_map<std::string, RobotConfigEntry> configs_;
    bool defaultSkipStaticTable_ = true;
    std::string defaultStaticTablePath_;
};

static int read_optional_map_id(const json& obj, const char* key, int def = 0) {
    auto it = obj.find(key);
    if (it == obj.end() || it->is_null()) return def;
    if (it->is_number_integer()) return it->get<int>();
    if (it->is_number_unsigned()) return static_cast<int>(it->get<unsigned int>());
    if (it->is_string()) {
        try { return std::stoi(it->get<std::string>()); } catch (...) { return def; }
    }
    return def;
}

static int read_task_map_id(const json& taskObj, int def = 0) {
    int mapId = read_optional_map_id(taskObj, "mapId", def);
    if (mapId > 0) return mapId;
    mapId = read_optional_map_id(taskObj, "map_id", def);
    return mapId > 0 ? mapId : def;
}

static int read_request_map_id(const json& root, int def = 0) {
    int mapId = read_optional_map_id(root, "mapId", def);
    if (mapId > 0) return mapId;
    if (root.contains("environmentContext") && root["environmentContext"].is_object()) {
        mapId = read_optional_map_id(root["environmentContext"], "mapId", def);
        if (mapId > 0) return mapId;
    }
    return def;
}

static int infer_task_map_id(const json& taskObj,
                             const json& requestRoot,
                             MultiMapManager& mapManager) {
    int mapId = read_task_map_id(taskObj, read_request_map_id(requestRoot, 0));
    if (mapId > 0) return mapId;
    std::string bindRobotId = read_string(taskObj, "bindRobotId", "");
    if (!bindRobotId.empty()) {
        auto mapped = mapManager.getDeviceMap(bindRobotId);
        if (mapped.has_value()) return *mapped;
    }
    std::vector<std::string> deviceIds;
    collect_vehicle_filters(taskObj, deviceIds);
    if (deviceIds.size() == 1) {
        auto mapped = mapManager.getDeviceMap(deviceIds.front());
        if (mapped.has_value()) return *mapped;
    }
    auto onlyCtx = mapManager.singleReadyContext();
    if (onlyCtx) {
        return onlyCtx->activeMapId.load();
    }
    return 0;
}

static int infer_status_map_id(const RobotStatusEntry& entry,
                               MultiMapManager& mapManager) {
    if (entry.mapId > 0) return entry.mapId;
    auto mapped = mapManager.getDeviceMap(entry.deviceId);
    if (mapped.has_value()) return *mapped;
    auto onlyCtx = mapManager.singleReadyContext();
    if (onlyCtx) return onlyCtx->activeMapId.load();
    return 0;
}

static int detect_map_id_from_map_message(const json& j) {
    const json* root = &j;
    if (j.is_object() && j.contains("mapData") && j["mapData"].is_object()) {
        root = &j["mapData"];
    }
    int mapId = read_optional_map_id(j, "mapId", 0);
    if (mapId > 0) return mapId;
    if (j.contains("info") && j["info"].is_object()) {
        mapId = read_optional_map_id(j["info"], "mapId", 0);
        if (mapId > 0) return mapId;
    }
    if (root->is_object()) {
        mapId = read_optional_map_id(*root, "mapId", 0);
        if (mapId > 0) return mapId;
        if (root->contains("info") && (*root)["info"].is_object()) {
            mapId = read_optional_map_id((*root)["info"], "mapId", 0);
            if (mapId > 0) return mapId;
        }
    }
    return 0;
}

static std::string detect_map_version_from_message(const json& j) {
    std::string version = read_string(j, "mapVersion", "");
    if (!version.empty()) return version;
    if (j.contains("environmentContext") && j["environmentContext"].is_object()) {
        version = read_string(j["environmentContext"], "mapVersion", "");
        if (!version.empty()) return version;
    }
    if (j.contains("mapData") && j["mapData"].is_object()) {
        version = read_string(j["mapData"], "mapVersion", "");
    }
    return version;
}

static json filter_agv_status_list_for_map(const json& root,
                                           int mapId,
                                           MultiMapManager& mapManager) {
    json out = json::array();
    if (!root.contains("agvStatusList") || !root["agvStatusList"].is_array()) return out;
    for (const auto& item : root["agvStatusList"]) {
        RobotStatusEntry entry;
        if (!fill_status_entry_from_json(item, entry)) continue;
        int resolved = infer_status_map_id(entry, mapManager);
        if (resolved != mapId) continue;
        json normalized = item;
        normalized["mapId"] = mapId;
        out.push_back(std::move(normalized));
    }
    return out;
}

static std::vector<std::pair<int, json>> split_scheduling_payloads_by_map(const json& root,
                                                                          MultiMapManager& mapManager) {
    std::unordered_map<int, json> tasksByMap;
    if (!root.contains("candidateTasks") || !root["candidateTasks"].is_array()) return {};
    for (const auto& task : root["candidateTasks"]) {
        int mapId = infer_task_map_id(task, root, mapManager);
        if (mapId <= 0) {
            auto onlyCtx = mapManager.singleReadyContext();
            if (onlyCtx) mapId = onlyCtx->activeMapId.load();
        }
        if (mapId <= 0) continue;
        json normalized = task;
        normalized["mapId"] = mapId;
        auto& arr = tasksByMap[mapId];
        if (!arr.is_array()) arr = json::array();
        arr.push_back(std::move(normalized));
    }

    std::vector<std::pair<int, json>> out;
    out.reserve(tasksByMap.size());
    for (auto& kv : tasksByMap) {
        json payload = root;
        payload["candidateTasks"] = kv.second;
        payload["taskNumber"] = static_cast<int>(kv.second.size());
        payload["mapId"] = kv.first;
        if (payload.contains("environmentContext") && payload["environmentContext"].is_object()) {
            payload["environmentContext"]["mapId"] = kv.first;
        }
        if (root.contains("agvStatusList") && root["agvStatusList"].is_array()) {
            payload["agvStatusList"] = filter_agv_status_list_for_map(root, kv.first, mapManager);
        }
        out.push_back({kv.first, std::move(payload)});
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    return out;
}

static std::string map_cache_path_for_id(const std::string& basePath, int mapId) {
    if (basePath.empty()) return basePath;
    std::filesystem::path path(basePath);
    if (mapId <= 0) return path.string();
    std::string stem = path.stem().string();
    std::string ext = path.extension().string();
    std::filesystem::path fileName = stem + "_map_" + std::to_string(mapId) + ext;
    if (path.has_parent_path()) {
        return (path.parent_path() / fileName).string();
    }
    return fileName.string();
}

static bool reload_context_map_payload(const json& j,
                                       MapRuntimeContext& ctx,
                                       const std::string& cacheBasePath,
                                       bool startupSummary) {
    int targetMapId = detect_map_id_from_map_message(j);
    if (targetMapId <= 0) {
        targetMapId = ctx.activeMapId.load();
    }
    if (targetMapId <= 0) targetMapId = ctx.mapId;

    auto nextMapInfo = std::make_unique<MapInfo>(2);
    std::unique_ptr<AStarPathFinder> nextAStar;
    NodeReservationTable stagingReservations;
    std::atomic<int> parsedMapId(targetMapId);
    std::string cachePath = map_cache_path_for_id(cacheBasePath, targetMapId);
    if (!handle_map_info(j, *nextMapInfo, stagingReservations, nextAStar, cachePath, parsedMapId)) {
        return false;
    }

    const int resolvedMapId = parsedMapId.load() > 0 ? parsedMapId.load() : targetMapId;
    const double nearConflictThresholdMm = std::max(0.0, reserve_near_conflict_threshold_mm());
    ctx.nodeReservations.configureDistanceConflict(*nextMapInfo, nearConflictThresholdMm);

    {
        std::unique_lock<std::shared_mutex> mapWriteLock(ctx.mapMutex);
        ctx.mapInfoPtr = std::move(nextMapInfo);
        ctx.aStarPtr = std::move(nextAStar);
        ctx.activeMapId.store(resolvedMapId);
        ctx.mapVersion = detect_map_version_from_message(j);
        ctx.mapReady.store(true);
        ctx.mapEpoch.fetch_add(1);
        if (!ctx.skipStaticTable) {
            ctx.skipStaticTable = true;
            ctx.staticTableReady = false;
            if (startupSummary) {
                std::cout << log_time_prefix()
                          << "[Map] disable static table after dynamic map reload mapId="
                          << resolvedMapId << std::endl;
            }
        }
    }
    ctx.robotRepo.clearDynamicStateKeepPlans();
    return true;
}

static void attach_map_metadata(ordered_json& body,
                                const MapRuntimeContext& ctx) {
    body["mapId"] = ctx.activeMapId.load();
    if (!ctx.mapVersion.empty()) {
        body["mapVersion"] = ctx.mapVersion;
    }
}

static long long current_millis() {
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return ms.count();
}

static std::string format_local_time_hhmmss_ms();

static void maybe_cleanup_reservations(NodeReservationTable& reservations) {
    const int intervalMs = getenv_int("RESERVE_CLEANUP_INTERVAL_MS", 1000);
    if (intervalMs <= 0) return;
    static std::atomic<long long> lastCleanupMs{0};
    const long long nowMs = current_millis();
    long long prev = lastCleanupMs.load(std::memory_order_relaxed);
    if (nowMs - prev < intervalMs) return;
    long long expected = prev;
    if (!lastCleanupMs.compare_exchange_strong(expected, nowMs, std::memory_order_relaxed)) return;
    reservations.cleanupExpired();
}

static void maybe_release_stale_reservations(RobotDataRepository& repo,
                                             NodeReservationTable& reservations) {
    const int staleMs = getenv_int("STATUS_STALE_RELEASE_MS", 0);
    if (staleMs <= 0) return;
    const int checkIntervalMs = std::max(500, std::min(5000, staleMs / 2));
    static std::atomic<long long> lastCheckMs{0};
    const long long nowMs = current_millis();
    long long prev = lastCheckMs.load(std::memory_order_relaxed);
    if (nowMs - prev < checkIntervalMs) return;
    long long expected = prev;
    if (!lastCheckMs.compare_exchange_strong(expected, nowMs, std::memory_order_relaxed)) return;
    const auto now = std::chrono::steady_clock::now();
    auto staleIds = repo.collectStaleDevices(staleMs, now);
    if (staleIds.empty()) return;
    const bool logSummary = env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL");
    for (const auto& deviceId : staleIds) {
        reservations.releaseAllByOwner(deviceId);
        repo.updateReservedNodesAndMismatchOnly(deviceId,
                                                std::vector<int>{},
                                                std::chrono::steady_clock::time_point{},
                                                -1);
        repo.updateLastSentRoute(deviceId, {});
        if (logSummary) {
            std::cout << log_time_prefix()
                      << "[Reserve] stale release deviceId=" << deviceId
                      << " staleMs=" << staleMs
                      << std::endl;
        }
    }
}

class SimPosePublisher {
public:
    explicit SimPosePublisher(const RabbitMQConfig& cfg);
    ~SimPosePublisher();
    bool publishPose(const nlohmann::json& body);
private:
    struct Impl {
        explicit Impl(const RabbitMQConfig& cfgIn);
        ~Impl();
        bool publish(const std::string& payload);
        void close();
        bool ensure_connected();
        RabbitMQConfig cfg;
        amqp_connection_state_t conn;
        bool connected;
        bool channel_open;
        std::string exchange;
        std::string exchange_type;
        std::string queue;
        std::string binding_key;
        std::string routing_key;
    };
    Impl* impl_;
};

SimPosePublisher::Impl::Impl(const RabbitMQConfig& cfgIn)
    : cfg(cfgIn),
      conn(nullptr),
      connected(false),
      channel_open(false) {
    exchange = getenv_str("SIM_POSE_EXCHANGE", "AlgoSimExchange");
    exchange_type = getenv_str("SIM_POSE_EXCHANGE_TYPE", "fanout");
    queue = getenv_str("SIM_POSE_QUEUE", "AlgoSimQueue");
    binding_key = getenv_str("SIM_POSE_BINDING_KEY", "#");
    routing_key = getenv_str("SIM_POSE_ROUTING_KEY", "SimPose");
}

SimPosePublisher::Impl::~Impl() {
    close();
}

bool SimPosePublisher::Impl::ensure_connected() {
    if (connected && conn) return true;
    close();
    conn = amqp_new_connection();
    amqp_socket_t* sock = amqp_tcp_socket_new(conn);
    if (!sock) {
        std::cerr << "[SimPose] cannot create TCP socket" << std::endl;
        close();
        return false;
    }
    if (amqp_socket_open(sock, cfg.host.c_str(), cfg.port)) {
        std::cerr << "[SimPose] socket open failed" << std::endl;
        close();
        return false;
    }
    if (!amqp_ok(amqp_login(conn, cfg.vhost.c_str(), 0, 131072, 0,
                            AMQP_SASL_METHOD_PLAIN,
                            cfg.username.c_str(), cfg.password.c_str()),
                 "SimPose login")) {
        close();
        return false;
    }
    amqp_channel_open(conn, 1);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "SimPose channel.open")) {
        close();
        return false;
    }
    channel_open = true;
    amqp_exchange_declare(conn, 1,
                          amqp_cstring_bytes(exchange.c_str()),
                          amqp_cstring_bytes(exchange_type.c_str()),
                          0, 1, 0, 0, amqp_empty_table);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "SimPose exchange.declare")) {
        close();
        return false;
    }
    amqp_queue_declare(conn, 1,
                       amqp_cstring_bytes(queue.c_str()),
                       0, 1, 0, 0, amqp_empty_table);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "SimPose queue.declare")) {
        close();
        return false;
    }
    amqp_queue_bind(conn, 1,
                    amqp_cstring_bytes(queue.c_str()),
                    amqp_cstring_bytes(exchange.c_str()),
                    amqp_cstring_bytes(binding_key.c_str()),
                    amqp_empty_table);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "SimPose queue.bind")) {
        close();
        return false;
    }
    connected = true;
    return true;
}

bool SimPosePublisher::Impl::publish(const std::string& payload) {
    if (!ensure_connected()) return false;
    amqp_bytes_t message_bytes;
    message_bytes.len = payload.size();
    message_bytes.bytes = (void*)payload.data();
    amqp_basic_properties_t props;
    std::memset(&props, 0, sizeof(props));
    props._flags = AMQP_BASIC_CONTENT_TYPE_FLAG | AMQP_BASIC_DELIVERY_MODE_FLAG;
    props.content_type = amqp_cstring_bytes("application/json");
    props.delivery_mode = 1;
    int rc = amqp_basic_publish(conn, 1,
                                amqp_cstring_bytes(exchange.c_str()),
                                amqp_cstring_bytes(routing_key.c_str()),
                                0, 0, &props, message_bytes);
    if (rc != 0) {
        std::cerr << "[SimPose] publish failed rc=" << rc << std::endl;
        close();
        return false;
    }
    return true;
}

void SimPosePublisher::Impl::close() {
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

SimPosePublisher::SimPosePublisher(const RabbitMQConfig& cfg)
    : impl_(new Impl(cfg)) {}

SimPosePublisher::~SimPosePublisher() {
    delete impl_;
    impl_ = nullptr;
}

bool SimPosePublisher::publishPose(const nlohmann::json& body) {
    if (!impl_) return false;
    return impl_->publish(body.dump());
}

class SimReservedPublisher {
public:
    explicit SimReservedPublisher(const RabbitMQConfig& cfg);
    ~SimReservedPublisher();
    bool publishSnapshot(const nlohmann::json& body);
private:
    struct Impl {
        explicit Impl(const RabbitMQConfig& cfgIn);
        ~Impl();
        bool publish(const std::string& payload);
        void close();
        bool ensure_connected();
        RabbitMQConfig cfg;
        amqp_connection_state_t conn;
        bool connected;
        bool channel_open;
        std::string exchange;
        std::string exchange_type;
        std::string queue;
        std::string binding_key;
        std::string routing_key;
    };
    Impl* impl_;
};

SimReservedPublisher::Impl::Impl(const RabbitMQConfig& cfgIn)
    : cfg(cfgIn),
      conn(nullptr),
      connected(false),
      channel_open(false) {
    exchange = getenv_str("SIM_RESERVED_EXCHANGE", "AlgoSimExchange");
    exchange_type = getenv_str("SIM_RESERVED_EXCHANGE_TYPE", "fanout");
    queue = getenv_str("SIM_RESERVED_QUEUE", "AlgoSimReservedQueue");
    binding_key = getenv_str("SIM_RESERVED_BINDING_KEY", "#");
    routing_key = getenv_str("SIM_RESERVED_ROUTING_KEY", "SimReserved");
}

SimReservedPublisher::Impl::~Impl() {
    close();
}

bool SimReservedPublisher::Impl::ensure_connected() {
    if (connected && conn) return true;
    close();
    conn = amqp_new_connection();
    amqp_socket_t* sock = amqp_tcp_socket_new(conn);
    if (!sock) {
        std::cerr << "[SimReserved] cannot create TCP socket" << std::endl;
        close();
        return false;
    }
    if (amqp_socket_open(sock, cfg.host.c_str(), cfg.port)) {
        std::cerr << "[SimReserved] socket open failed" << std::endl;
        close();
        return false;
    }
    if (!amqp_ok(amqp_login(conn, cfg.vhost.c_str(), 0, 131072, 0,
                            AMQP_SASL_METHOD_PLAIN,
                            cfg.username.c_str(), cfg.password.c_str()),
                 "SimReserved login")) {
        close();
        return false;
    }
    amqp_channel_open(conn, 1);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "SimReserved channel.open")) {
        close();
        return false;
    }
    channel_open = true;
    amqp_exchange_declare(conn, 1,
                          amqp_cstring_bytes(exchange.c_str()),
                          amqp_cstring_bytes(exchange_type.c_str()),
                          0, 1, 0, 0, amqp_empty_table);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "SimReserved exchange.declare")) {
        close();
        return false;
    }
    amqp_queue_declare(conn, 1,
                       amqp_cstring_bytes(queue.c_str()),
                       0, 1, 0, 0, amqp_empty_table);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "SimReserved queue.declare")) {
        close();
        return false;
    }
    amqp_queue_bind(conn, 1,
                    amqp_cstring_bytes(queue.c_str()),
                    amqp_cstring_bytes(exchange.c_str()),
                    amqp_cstring_bytes(binding_key.c_str()),
                    amqp_empty_table);
    if (!amqp_ok(amqp_get_rpc_reply(conn), "SimReserved queue.bind")) {
        close();
        return false;
    }
    connected = true;
    return true;
}

bool SimReservedPublisher::Impl::publish(const std::string& payload) {
    if (!ensure_connected()) return false;
    amqp_bytes_t message_bytes;
    message_bytes.len = payload.size();
    message_bytes.bytes = (void*)payload.data();
    amqp_basic_properties_t props;
    std::memset(&props, 0, sizeof(props));
    props._flags = AMQP_BASIC_CONTENT_TYPE_FLAG | AMQP_BASIC_DELIVERY_MODE_FLAG;
    props.content_type = amqp_cstring_bytes("application/json");
    props.delivery_mode = 1;
    int rc = amqp_basic_publish(conn, 1,
                                amqp_cstring_bytes(exchange.c_str()),
                                amqp_cstring_bytes(routing_key.c_str()),
                                0, 0, &props, message_bytes);
    if (rc != 0) {
        std::cerr << "[SimReserved] publish failed rc=" << rc << std::endl;
        close();
        return false;
    }
    return true;
}

void SimReservedPublisher::Impl::close() {
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

SimReservedPublisher::SimReservedPublisher(const RabbitMQConfig& cfg)
    : impl_(new Impl(cfg)) {}

SimReservedPublisher::~SimReservedPublisher() {
    delete impl_;
    impl_ = nullptr;
}

bool SimReservedPublisher::publishSnapshot(const nlohmann::json& body) {
    if (!impl_) return false;
    return impl_->publish(body.dump());
}

// Forward declarations for helper utilities defined later in this file.
static std::string read_string(const json& obj, const char* key, const std::string& def);
static int read_int(const json& obj, const char* key, int def);
static double read_double(const json& obj, const char* key, double def);
static std::string normalize_request_timestamp(const std::string& raw);
static int resolve_status_node_id(const RobotStatusEntry& st, const MapInfo& mapInfo);
static int resolve_status_next_node_id(const RobotStatusEntry& st, const MapInfo& mapInfo);
static int resolve_committed_next_node_id(const RobotStatusEntry& st, const MapInfo& mapInfo);
static bool fill_status_entry_from_json(const json& item, RobotStatusEntry& entry);
static void collect_vehicle_filters(const json& taskObj, std::vector<std::string>& out);
static size_t cache_agv_status_list(const json& j, RobotDataRepository& repo);
static std::string normalize_alloc_algo(const std::string& raw);
static std::string read_alloc_algo_from_json(const json& j);
static Amr build_amr_from_status(const RobotStatusEntry& s,
                                 int resolvedAheadMs,
                                 int resolvedDetourMm,
                                 const MapInfo& mapInfo);
static size_t count_assigned_tasks(const AllocationResult& result);
static std::string iso8601_utc_now();
static std::string format_local_time();
static std::string log_time_prefix();
class TrafficDebugManager;
static ordered_json build_traffic_path_payload(
    const std::string& messageId,
    int mapId,
    RobotDataRepository& repo,
    const MapInfo& mapInfo,
    const std::unordered_map<std::string, int>& taskPriorityById,
    bool mapReady,
    bool staticTableReady,
    bool skipStaticTable,
    StaticPathTable& staticTable,
    AStarPathFinder* aStarPtr,
    NodeReservationTable& reservations);
static void publish_trail_for_all_plans(
    const std::string& messageIdPrefix,
    RobotDataRepository& robotRepo,
    const MapInfo& mapInfo,
    std::shared_mutex& mapMutex,
    bool mapReady,
    bool skipStaticTable,
    bool staticTableReady,
    StaticPathTable& staticTable,
    AStarPathFinder* aStarPtr,
    NodeReservationTable& nodeReservations,
    AlgoPublisher& algoPublisher,
    bool logSummary,
    bool logDetail);

struct TrafficPayloadSummary {
    size_t pathInfoCount = 0;
    size_t pathPointCount = 0;
    size_t deviceCount = 0;
};

static TrafficPayloadSummary summarize_traffic_payload(const ordered_json& payload) {
    TrafficPayloadSummary summary;
    if (!payload.is_object()) return summary;
    auto it = payload.find("pathInfos");
    if (it == payload.end() || !it->is_array()) return summary;
    summary.pathInfoCount = it->size();
    std::unordered_set<std::string> devices;
    devices.reserve(it->size());
    for (const auto& info : *it) {
        if (!info.is_object()) continue;
        auto devIt = info.find("deviceId");
        if (devIt != info.end() && devIt->is_string()) {
            devices.insert(devIt->get<std::string>());
        }
        auto ptsIt = info.find("pathPoints");
        if (ptsIt != info.end() && ptsIt->is_array()) {
            summary.pathPointCount += ptsIt->size();
        }
    }
    summary.deviceCount = devices.size();
    return summary;
}

static std::string format_traffic_nodes_log(const ordered_json& payload) {
    if (!payload.is_object()) return std::string();
    auto infosIt = payload.find("pathInfos");
    if (infosIt == payload.end() || !infosIt->is_array()) return std::string();
    std::ostringstream oss;
    size_t printed = 0;
    bool firstLine = true;
    auto append_line = [&](const std::string& line) {
        if (!firstLine) {
            oss << "\n";
        }
        oss << line;
        firstLine = false;
    };
    for (const auto& info : *infosIt) {
        if (!info.is_object()) continue;
        std::string deviceId;
        long long pathId = -1;
        auto devIt = info.find("deviceId");
        if (devIt != info.end()) {
            if (devIt->is_string()) {
                deviceId = devIt->get<std::string>();
            } else if (devIt->is_number_integer()) {
                deviceId = std::to_string(devIt->get<long long>());
            }
        }
        auto pathIt = info.find("pathId");
        if (pathIt != info.end() && pathIt->is_number_integer()) {
            pathId = pathIt->get<long long>();
        }
        auto ptsIt = info.find("pathPoints");
        if (ptsIt == info.end() || !ptsIt->is_array()) continue;
        {
            std::ostringstream line;
            line << log_time_prefix() << "[TrafficPath]"
                 << " deviceId=" << (deviceId.empty() ? "<none>" : deviceId)
                 << " pathId=" << pathId
                 << " nodeCount=" << ptsIt->size();
            append_line(line.str());
        }
        for (const auto& pt : *ptsIt) {
            if (!pt.is_object()) continue;
            int nodeId = -1;
            int x = 0;
            int y = 0;
            auto nodeIt = pt.find("nodeId");
            if (nodeIt != pt.end() && nodeIt->is_number_integer()) nodeId = nodeIt->get<int>();
            auto xIt = pt.find("x");
            if (xIt != pt.end() && xIt->is_number_integer()) x = xIt->get<int>();
            auto yIt = pt.find("y");
            if (yIt != pt.end() && yIt->is_number_integer()) y = yIt->get<int>();
            std::ostringstream line;
            line << log_time_prefix() << "node[" << printed << "] = " << nodeId
                 << " , x=" << x << " , y=" << y;
            append_line(line.str());
            ++printed;
        }
    }
    return oss.str();
}

static std::string filename_timestamp_local_ms() {
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    std::time_t t = static_cast<std::time_t>(ms.count() / 1000);
    int msPart = static_cast<int>(ms.count() % 1000);
    if (msPart < 0) {
        msPart += 1000;
        t -= 1;
    }
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d-%02d%02d%02d-%03d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec, msPart);
    return std::string(buf);
}

static std::string make_traffic_filename(const std::string& prefix, const std::string& ext) {
    static std::atomic<unsigned long long> seq{0};
    unsigned long long id = seq.fetch_add(1);
    std::string cleanedExt = ext;
    if (!cleanedExt.empty() && cleanedExt.front() == '.') {
        cleanedExt.erase(cleanedExt.begin());
    }
    std::string name = prefix + "_" + filename_timestamp_local_ms() + "_" + std::to_string(id);
    if (!cleanedExt.empty()) {
        name += "." + cleanedExt;
    }
    return name;
}

static std::string trim_ascii(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) {
        ++start;
    }
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        --end;
    }
    return s.substr(start, end - start);
}

static std::string normalize_alloc_algo(const std::string& raw) {
    std::string trimmed = trim_ascii(raw);
    if (trimmed.empty()) return trimmed;
    return to_lower_ascii(std::move(trimmed));
}

static std::string read_alloc_algo_from_json(const json& j) {
    static const char* keys[] = {
        "allocationAlgorithm",
        "allocationAlgo",
        "allocAlgo",
        "algorithm",
        "algo",
        "allocator",
    };
    for (const char* key : keys) {
        if (j.contains(key) && j[key].is_string()) {
            return trim_ascii(j[key].get<std::string>());
        }
    }
    if (j.contains("environmentContext") && j["environmentContext"].is_object()) {
        const auto& env = j["environmentContext"];
        for (const char* key : keys) {
            if (env.contains(key) && env[key].is_string()) {
                return trim_ascii(env[key].get<std::string>());
            }
        }
    }
    return "";
}

static std::string make_unknown_rk_filename() {
    static std::atomic<unsigned long long> seq{0};
    unsigned long long id = seq.fetch_add(1);
    return "unknown_rk_" + filename_timestamp_local_ms() + "_" + std::to_string(id) + ".json";
}

static void dump_unknown_message(const std::string& dir, const std::string& payload) {
    if (dir.empty()) return;
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::path p(dir);
    if (!fs::exists(p, ec)) {
        ec.clear();
        if (!fs::create_directories(p, ec) && ec) {
            std::cerr << log_time_prefix() << "[UnknownRK] failed to create directory "
                      << dir << " error=" << ec.message() << std::endl;
            return;
        }
    }
    fs::path filePath = p / make_unknown_rk_filename();
    std::ofstream ofs(filePath, std::ios::out | std::ios::binary);
    if (!ofs) {
        std::cerr << log_time_prefix() << "[UnknownRK] cannot open file "
                  << filePath << std::endl;
        return;
    }
    ofs << payload;
    ofs.close();
    std::cout << log_time_prefix() << "[UnknownRK] dumped message to "
              << filePath.string() << std::endl;
}

static bool is_known_routing_key(const std::string& rk) {
    static const std::unordered_set<std::string> known{
        "AssignmentTaskRequest",
        "SendMapInfo",
        "SendRobotStatusInfos",
        "SendRobotConfigInfos",
        "RobotPathRequest",
        "RobotTrailRequest",
        "RobotAroundPathRequest"
    };
    return !rk.empty() && known.find(rk) != known.end();
}

static void append_unique(std::vector<std::string>& list, const std::string& value) {
    if (value.empty()) return;
    if (std::find(list.begin(), list.end(), value) != list.end()) return;
    list.push_back(value);
}

static bool queue_exists(amqp_connection_state_t conn, int channelId, const std::string& queueName) {
    if (queueName.empty()) return false;
    // passive=1, durable=1 (与声明时一致)，仅检查存在性，不创建。
    amqp_queue_declare(conn, channelId,
                       amqp_cstring_bytes(queueName.c_str()),
                       1, 1, 0, 0, amqp_empty_table);
    amqp_rpc_reply_t rep = amqp_get_rpc_reply(conn);
    return rep.reply_type == AMQP_RESPONSE_NORMAL;
}

static void purge_queue(amqp_connection_state_t conn, int channelId, const std::string& queueName) {
    if (queueName.empty()) return;
    if (!queue_exists(conn, channelId, queueName)) {
        std::cout << log_time_prefix() << "[MQ] skip purge (queue not found) queue='"
                  << queueName << "'" << std::endl;
        return;
    }
    amqp_queue_purge(conn, channelId, amqp_cstring_bytes(queueName.c_str()));
    amqp_rpc_reply_t rep = amqp_get_rpc_reply(conn);
    if (rep.reply_type == AMQP_RESPONSE_NORMAL) {
        int purgedCount = -1;
        if (rep.reply.id == AMQP_QUEUE_PURGE_OK_METHOD && rep.reply.decoded) {
            auto* ok = static_cast<amqp_queue_purge_ok_t*>(rep.reply.decoded);
            purgedCount = static_cast<int>(ok->message_count);
        }
        if (purgedCount >= 0) {
            std::cout << log_time_prefix() << "[MQ] purge queue='" << queueName
                      << "' count=" << purgedCount << std::endl;
        } else {
            std::cout << log_time_prefix() << "[MQ] purged queue='" << queueName << "'" << std::endl;
        }
        return;
    }
    std::cerr << log_time_prefix() << "[MQ] purge failed queue='" << queueName << "'" << std::endl;
    amqp_ok(rep, "queue.purge");
}

static void purge_startup_queues(amqp_connection_state_t conn,
                                 const std::vector<std::string>& queues) {
    for (const auto& q : queues) {
        if (q.empty()) continue;
        const int purgeChannel = 2;
        amqp_channel_open(conn, purgeChannel);
        if (!amqp_ok(amqp_get_rpc_reply(conn), "purge channel.open")) {
            continue;
        }
        purge_queue(conn, purgeChannel, q);
        amqp_channel_close(conn, purgeChannel, AMQP_REPLY_SUCCESS);
        amqp_get_rpc_reply(conn);
    }
}

static void prune_files_by_count(const std::filesystem::path& dir,
                                 const std::string& prefix,
                                 const std::string& ext,
                                 int keepMax) {
    if (keepMax <= 0) return;
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::exists(dir, ec) || ec) return;

    struct Entry {
        fs::file_time_type ts;
        fs::path path;
    };
    std::vector<Entry> entries;
    std::string prefixTag = prefix + "_";
    std::string extTag = ext.empty() ? std::string() : ("." + ext);
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        if (ec) break;
        if (!entry.is_regular_file(ec) || ec) { ec.clear(); continue; }
        fs::path p = entry.path();
        if (!extTag.empty() && p.extension() != extTag) continue;
        std::string name = p.filename().string();
        if (name.rfind(prefixTag, 0) != 0) continue;
        fs::file_time_type ts = entry.last_write_time(ec);
        if (ec) { ec.clear(); continue; }
        entries.push_back(Entry{ts, std::move(p)});
    }
    std::sort(entries.begin(), entries.end(),
              [](const Entry& a, const Entry& b) { return a.ts > b.ts; });
    for (size_t i = static_cast<size_t>(keepMax); i < entries.size(); ++i) {
        fs::remove(entries[i].path, ec);
        if (ec) ec.clear();
    }
}

class TrafficDebugManager {
public:
    TrafficDebugManager() = default;
    TrafficDebugManager(const TrafficDebugManager&) = delete;
    TrafficDebugManager& operator=(const TrafficDebugManager&) = delete;

    void configure(const std::string& dumpDir,
                   int dumpKeepMax,
                   bool logSummary,
                   bool logJson,
                   bool logDetail,
                   bool dumpEnabled,
                   const std::string& lineLogPath) {
        std::lock_guard<std::mutex> lock(mu_);
        dumpDir_ = dumpDir;
        dumpKeepMax_ = dumpKeepMax;
        dumpEnabled_ = dumpEnabled && !dumpDir_.empty();
        logSummary_ = logSummary;
        logJson_ = logJson;
        logDetail_ = logDetail;
        lineLogPath_ = lineLogPath;
        lineLogEnabled_ = !lineLogPath_.empty();
        if (lineLogEnabled_) {
            std::filesystem::path p(lineLogPath_);
            std::string parent = p.has_parent_path() ? p.parent_path().string() : std::string();
            if (!parent.empty()) {
                ensure_dir_locked(parent, "traffic path line log directory");
            }
        }
        if (dumpEnabled_ && dumpKeepMax_ > 0) {
            std::filesystem::path dirPath(dumpDir_);
            ensure_dir_locked(dirPath.string(), "traffic dump directory");
            prune_files_by_count(dirPath, "traffic_path", "json", dumpKeepMax_);
        }
    }

    void onPublish(const ordered_json& payload,
                   const std::string& payloadStr,
                   const char* source) {
        const bool needLog = logSummary_ || logJson_ || logDetail_;
        const bool needDump = dumpEnabled_;
        const bool needLineLog = lineLogEnabled_;
        if (!needLog && !needDump && !needLineLog) return;

        TrafficPayloadSummary summary;
        std::string messageId;
        int mapId = -1;
        std::string detailLog;
        std::vector<std::string> lineLogs;
        if (needLineLog) {
            auto read_string_field = [](const ordered_json& obj, const char* key) -> std::string {
                auto it = obj.find(key);
                if (it == obj.end() || it->is_null()) return std::string();
                if (it->is_string()) return it->get<std::string>();
                if (it->is_number_integer()) return std::to_string(it->get<long long>());
                if (it->is_number_unsigned()) return std::to_string(it->get<unsigned long long>());
                return std::string();
            };
            struct PathLineEntry {
                std::string pathId;
                std::vector<std::string> nodes;
            };
            std::unordered_map<std::string, PathLineEntry> perDevice;
            std::vector<std::string> order;
            if (payload.is_object()) {
                auto pit = payload.find("pathInfos");
                if (pit != payload.end() && pit->is_array()) {
                    const std::string timeTag = format_local_time_hhmmss_ms();
                    for (const auto& info : *pit) {
                        if (!info.is_object()) continue;
                        std::string devId = read_string_field(info, "deviceId");
                        if (devId.empty()) continue;
                        auto it = perDevice.find(devId);
                        if (it == perDevice.end()) {
                            order.push_back(devId);
                            it = perDevice.emplace(devId, PathLineEntry{}).first;
                        }
                        PathLineEntry& entry = it->second;
                        if (entry.pathId.empty()) {
                            entry.pathId = read_string_field(info, "pathId");
                        }
                        auto ptsIt = info.find("pathPoints");
                        if (ptsIt != info.end() && ptsIt->is_array()) {
                            for (const auto& pt : *ptsIt) {
                                if (!pt.is_object()) continue;
                                std::string nodeId = read_string_field(pt, "nodeId");
                                if (nodeId.empty()) continue;
                                if (!entry.nodes.empty() && entry.nodes.back() == nodeId) continue;
                                entry.nodes.push_back(std::move(nodeId));
                            }
                        }
                        if (entry.nodes.empty()) {
                            auto startIt = info.find("startPoint");
                            if (startIt != info.end() && startIt->is_object()) {
                                std::string nodeId = read_string_field(*startIt, "nodeId");
                                if (!nodeId.empty()) entry.nodes.push_back(std::move(nodeId));
                            }
                        }
                    }
                    for (const auto& devId : order) {
                        auto it2 = perDevice.find(devId);
                        if (it2 == perDevice.end()) continue;
                        const auto& entry = it2->second;
                        if (entry.nodes.empty()) continue;
                        std::ostringstream oss;
                        oss << (entry.pathId.empty() ? "0" : entry.pathId)
                            << "(" << devId << ", " << timeTag << "):";
                        for (size_t i = 0; i < entry.nodes.size(); ++i) {
                            if (i > 0) oss << "-";
                            oss << entry.nodes[i];
                        }
                        lineLogs.push_back(oss.str());
                    }
                }
            }
        }
        if (needLog) {
            summary = summarize_traffic_payload(payload);
            if (payload.is_object()) {
                auto msgIt = payload.find("messageId");
                if (msgIt != payload.end()) {
                    if (msgIt->is_string()) {
                        messageId = msgIt->get<std::string>();
                    } else if (msgIt->is_number_integer()) {
                        messageId = std::to_string(msgIt->get<long long>());
                    }
                }
                auto mapIt = payload.find("mapId");
                if (mapIt != payload.end() && mapIt->is_number_integer()) {
                    mapId = mapIt->get<int>();
                }
            }
            if (logDetail_) {
                detailLog = format_traffic_nodes_log(payload);
            }
        }
        const size_t bytes = payloadStr.size();
        std::lock_guard<std::mutex> lock(mu_);
        if (needLog) {
            log_payload_locked(payloadStr, source ? source : "",
                               messageId, mapId, summary, bytes, detailLog);
        }
        if (needDump) {
            dump_payload_locked(payloadStr);
        }
        if (needLineLog && !lineLogs.empty()) {
            for (const auto& line : lineLogs) {
                append_line_log_locked(line);
            }
        }
    }

private:
    bool ensure_dir_locked(const std::string& dir, const char* label) {
        if (dir.empty()) return false;
        namespace fs = std::filesystem;
        std::error_code ec;
        fs::path p(dir);
        if (fs::exists(p, ec)) {
            if (ec) ec.clear();
            return true;
        }
        if (fs::create_directories(p, ec)) return true;
        if (ec) {
            std::cerr << log_time_prefix() << "[TrafficDebug] failed to create "
                      << label << ": " << dir << " error=" << ec.message() << std::endl;
        }
        return false;
    }

    void dump_payload_locked(const std::string& payload) {
        if (!dumpEnabled_) return;
        std::filesystem::path dirPath(dumpDir_);
        if (!ensure_dir_locked(dirPath.string(), "traffic dump directory")) return;
        namespace fs = std::filesystem;
        fs::path filePath = dirPath / make_traffic_filename("traffic_path", "json");
        std::ofstream ofs(filePath, std::ios::out | std::ios::binary);
        if (!ofs) {
            std::cerr << log_time_prefix() << "[TrafficDump] cannot open file "
                      << filePath << std::endl;
            return;
        }
        ofs << payload;
        ofs.close();
        if (dumpKeepMax_ > 0) {
            prune_files_by_count(dirPath, "traffic_path", "json", dumpKeepMax_);
        }
    }

    void append_line_log_locked(const std::string& line) {
        if (!lineLogEnabled_ || lineLogPath_.empty()) return;
        std::filesystem::path filePath(lineLogPath_);
        std::ofstream ofs(filePath, std::ios::out | std::ios::binary | std::ios::app);
        if (!ofs) {
            std::cerr << log_time_prefix() << "[TrafficLine] cannot open file "
                      << filePath << std::endl;
            return;
        }
        ofs << line << "\n";
    }

    void log_payload_locked(const std::string& payload,
                            const std::string& source,
                            const std::string& messageId,
                            int mapId,
                            const TrafficPayloadSummary& summary,
                            size_t bytes,
                            const std::string& detailLog) {
        std::ostringstream oss;
        oss << log_time_prefix() << "[TrafficPath]"
            << " source=" << (source.empty() ? "<none>" : source)
            << " messageId=" << (messageId.empty() ? "<none>" : messageId)
            << " mapId=" << mapId
            << " devices=" << summary.deviceCount
            << " paths=" << summary.pathInfoCount
            << " points=" << summary.pathPointCount
            << " bytes=" << bytes;
        if (logDetail_ && !detailLog.empty()) {
            oss << "\n" << detailLog;
        }
        if (logJson_) {
            oss << "\n" << payload;
        }
        oss << "\n";
        std::cout << oss.str();
    }

    std::mutex mu_;
    std::string dumpDir_;
    int dumpKeepMax_ = 0;
    bool dumpEnabled_ = false;
    bool logSummary_ = false;
    bool logJson_ = false;
    bool logDetail_ = false;
    std::string lineLogPath_;
    bool lineLogEnabled_ = false;
};

static SchedulingRequest build_minimal_request(const json& j) {
    SchedulingRequest req;
    req.messageId = read_string(j, "schedulingRequestId", "");
    if (req.messageId.empty()) {
        req.messageId = read_string(j, "messageId", "");
    }
    if (req.messageId.empty()) {
        req.messageId = iso8601_utc_now();
    }
    req.mapId = read_request_map_id(j, 0);
    req.mapVersion = detect_map_version_from_message(j);
    return req;
}

static void publish_rejected_allocation(ResultPublisher& resultPublisher,
                                        const json& requestJson,
                                        const std::string& reason,
                                        int status_code) {
    SchedulingRequest req = build_minimal_request(requestJson);
    const bool logReject = env_enabled("RECEIVER_LOG_REJECT_RESP");
    if (logReject) {
        std::cout << log_time_prefix() << "[AllocResp] reject messageId=" << req.messageId
                  << " status=" << status_code;
        if (!reason.empty()) {
            std::cout << " reason=" << reason;
        }
        std::cout << std::endl;
    }
    AllocationResult emptyResult;
    std::vector<Amr> emptyAmrs;
    std::vector<Task> emptyTasks;
    std::vector<std::string> constraints;
    if (!reason.empty()) {
        constraints.push_back(reason);
    }
    auto sent = resultPublisher.publish(emptyResult, emptyAmrs, emptyTasks,
                                        nullptr, 0, 0.0, constraints,
                                        &req, nullptr, status_code);
    if (!sent.has_value()) {
        std::cout << "[Warn] 忙碌状态响应发送失败 messageId=" << req.messageId << std::endl;
    } else if (logReject) {
        std::cout << "[Info] 忙碌状态已回复 messageId=" << req.messageId << std::endl;
    }
}

static bool should_log_replan_unreachable() {
    if (env_enabled("REPLAN_PROFILE")) return true;
    if (env_enabled("REPLAN_PROFILE_DETAIL")) return true;
    return env_enabled("REPLAN_LOG_UNREACHABLE");
}

static bool should_log_trail_missing_subtask(const std::string& deviceId,
                                             const std::string& subTaskId,
                                             bool logDetail) {
    if (logDetail) return true;
    if (!env_enabled("RECEIVER_LOG_ROUTE_WARN")) return false;
    const int intervalMs = std::max(0, getenv_int("RECEIVER_LOG_ROUTE_WARN_INTERVAL_MS", 5000));
    if (intervalMs == 0) return true;
    const std::string key = !deviceId.empty() ? deviceId : subTaskId;
    const long long nowMs = current_millis();
    static std::mutex mu;
    static std::unordered_map<std::string, long long> lastLogMs;
    std::lock_guard<std::mutex> lk(mu);
    auto it = lastLogMs.find(key);
    if (it != lastLogMs.end() && (nowMs - it->second) < intervalMs) {
        return false;
    }
    lastLogMs[key] = nowMs;
    return true;
}

static PathPlanningHelper::AmrPlanInfo compute_dynamic_plan_to_next_target(
    const std::string& deviceId,
    int startNodeId,
    int committedNextNodeId,
    int batteryLevel,
    int deviceType,
    const PlanCacheEntry& planEntry,
    RobotDataRepository& repo,
    const MapInfo& mapInfo,
    bool skipStaticTable,
    bool staticTableReady,
    StaticPathTable& staticTable,
    AStarPathFinder& aStar,
    NodeReservationTable& reservations,
    int reserveBudgetOverride,
    bool allowLongerRoute,
    ReplanStage stage,
    bool* updatedOut,
    bool* usedTempGoalOut = nullptr);

static void processSchedulingMessage(
    json payload,
    RobotDataRepository& robotRepo,
    NodeReservationTable& nodeReservations,
    MapInfo& mapInfo,
    std::shared_mutex& mapMutex,
    std::atomic<bool>& mapReady,
    std::atomic<int>& mapId,
    std::unique_ptr<AStarPathFinder>& aStarPtr,
    StaticPathTable& staticTable,
    bool staticTableReady,
    bool skipStaticTable,
    ResultPublisher& resultPublisher,
    AlgoPublisher& algoPublisher,
    TrafficPathPublisher& trafficPathPublisher,
    TrafficDebugManager& trafficDebug,
    const std::string& serviceName,
    int resolvedAheadMs,
    int resolvedDetourMm) {
    SchedulingRequest req;
    json j = std::move(payload);
    req.messageId = read_string(j, "schedulingRequestId", "");
    {
        std::string tsRaw = read_string(j, "requestTimestamp", "");
        req.requestTimestamp = normalize_request_timestamp(tsRaw);
    }
    req.requestType = parse_request_type(j);
    req.taskNumber = read_int(j, "taskNumber", 0);
    req.timeoutMs = read_int(j, "timeoutMs", 0);
    if (j.contains("triggerContext")) {
        const auto& tc = j["triggerContext"];
        if (tc.is_string()) {
            req.triggerContext = tc.get<std::string>();
        } else if (tc.is_object()) {
            if (tc.contains("type")) {
                req.triggerContext = read_string(tc, "type", "");
            } else if (tc.contains("triggerType")) {
                req.triggerContext = read_string(tc, "triggerType", "");
            }
        }
    }
    if (j.contains("triggerAgvId") && j["triggerAgvId"].is_array()) {
        for (const auto& s : j["triggerAgvId"]) { try { req.triggerAgvId.push_back(s.get<std::string>());} catch(...){} }
    }
    req.batchConfig = parse_batch_config(j);
    if (j.contains("environmentContext") && j["environmentContext"].is_object()) {
        const auto& env = j["environmentContext"];
        req.mapVersion = read_string(env, "mapVersion", "");
        req.mapId = read_optional_map_id(env, "mapId", req.mapId);
        req.systemLoad = read_int(env, "systemLoad", 0);
        req.systemLoadFactor = read_double(env, "systemLoadFactor", 0.0);
    }
    req.mapId = read_request_map_id(j, req.mapId);
    std::string reqAlgo = read_alloc_algo_from_json(j);
    std::string envAlgo = getenv_str("ALLOC_ALGO", "");
    if (envAlgo.empty()) {
        envAlgo = getenv_str("ALLOC_ALGORITHM", "");
    }
    std::string chosenAlgo = envAlgo.empty() ? reqAlgo : envAlgo;
    req.allocationAlgorithm = normalize_alloc_algo(chosenAlgo);
    const bool logAllocSummary = env_enabled("RECEIVER_LOG_ALLOC_SUMMARY");
    const bool logAllocDiag = env_enabled("RECEIVER_LOG_ALLOC_DIAG");
    const bool logAllocProfile = env_enabled("RECEIVER_LOG_ALLOC_PROFILE");
    const bool logAllocFullResult = env_enabled("RECEIVER_LOG_ALLOC_RESULT");
    const bool logSchedSummary = env_enabled("RECEIVER_LOG_SCHED_REQUEST");
    const bool logSchedTaskDetail = env_enabled("RECEIVER_LOG_SCHED_TASK_DETAIL");
    if (logSchedSummary || logSchedTaskDetail) {
        std::cout << "SchedulingRequestId=" << req.messageId
                  << " taskNumber=" << req.taskNumber
                  << " timeoutMs=" << req.timeoutMs;
        if (req.requestType != TaskFieldUtils::REQUEST_TYPE_UNKNOWN) {
            std::cout << " requestType=" << req.requestType
                      << "(" << TaskFieldUtils::DescribeRequestType(req.requestType) << ")";
        }
        if (!req.requestTimestamp.empty()) std::cout << " ts=" << req.requestTimestamp;
        if (!req.allocationAlgorithm.empty()) std::cout << " algo=" << req.allocationAlgorithm;
        std::cout << std::endl;
    }

    // 时间戳可信性校验（仅当消息携带 requestTimestamp 时启用）
    if (!req.requestTimestamp.empty()) {
        double limitSec = TaskAllocationUtils::getMaxTimestampSkewSec(0.5);
        double deltaSec = 0.0;
        bool okTs = TaskAllocationUtils::isTimestampTrusted(req.requestTimestamp, limitSec, &deltaSec);
        if (!okTs) {
            std::cout << "[警告] 通信不可信：消息时间与本地时间偏差="
                      << deltaSec << "s，超过阈值=" << limitSec << "s，丢弃消息，不进行分配。" << std::endl;
            std::cout << "提示：可通过环境变量 TS_MAX_SKEW_SEC 调整阈值（当前默认0.5s）。" << std::endl;
            return;
        }
    }
    if (logSchedTaskDetail) {
        const auto& arr = j["candidateTasks"];
        for (const auto& ct : arr) {
            std::string tid = read_string(ct, "taskId", "");
            // 仅用于调试打印：仅取小写 priority，大写 Priority 忽略
            int pri = read_int(ct, "priority", 0);
            int minBat = read_int(ct, "minBatteryLevel", 0);
            std::string bindRobotId = read_string(ct, "bindRobotId", "");
            std::vector<std::string> vtypes;
            if (bindRobotId.empty()) {
                collect_vehicle_filters(ct, vtypes);
            }
            std::string agvReqDesc;
            if (bindRobotId.empty()) {
                if (vtypes.empty()) {
                    agvReqDesc = "(all)";
                } else {
                    for (size_t i = 0; i < vtypes.size(); ++i) {
                        if (i > 0) agvReqDesc += ",";
                        agvReqDesc += vtypes[i];
                    }
                }
            } else {
                agvReqDesc = "(ignored)";
            }
            std::cout << "  [TASK] id=" << tid << " pri=" << pri << " minBattery=" << minBat
                      << " bindRobotId=" << (bindRobotId.empty() ? std::string("(none)") : bindRobotId)
                      << " agvRequirements="
                      << agvReqDesc
                      << std::endl;
            // 打印子任务点并转换为段（start->end），段动作时间取下一点estimatedDuration
            if (ct.contains("subTasks") && ct["subTasks"].is_array()) {
                const auto& subs = ct["subTasks"];
                std::vector<int> nodes; nodes.reserve(subs.size());
                std::vector<std::int64_t> estAtPoint; estAtPoint.reserve(subs.size());
                for (const auto& st : subs) {
                    std::string subTaskId = read_string(st, "subTaskId", "");
                    int nodeId = -1;
                    if (st.contains("point") && st["point"].is_object()) {
                        const auto& point = st["point"];
                        nodeId = read_int(point, "nodeId", -1);
                    }
                    nodes.push_back(nodeId);
                    std::int64_t estMs = 0;
                    try {
                        estMs = st.contains("estimatedDuration") ? st["estimatedDuration"].get<std::int64_t>() : 0;
                    } catch (...) {
                        double tmp = read_double(st, "estimatedDuration", 0.0);
                        estMs = static_cast<std::int64_t>(std::llround(tmp));
                    }
                    estAtPoint.push_back(estMs);
                    int pointTypeCode = parse_point_type(st);
                    std::cout << "    - point nodeId=" << nodeId
                              << " type=" << pointTypeCode
                              << "(" << TaskFieldUtils::DescribePointType(pointTypeCode) << ")"
                              << " subTaskId=" << (subTaskId.empty() ? std::string("<none>") : subTaskId)
                              << " est=" << (estAtPoint.back() / 1000.0) << "s"
                              << std::endl;
                }
                for (size_t i=0;i+1<nodes.size();++i) {
                    double segAction = estAtPoint[i+1] / 1000.0;
                    std::cout << "      segment " << nodes[i] << " -> " << nodes[i+1]
                              << " (action est=" << segAction << "s)" << std::endl;
                }
            }
        }
    }

    // Debug block removed here; moved after taskList is constructed

    bool hasCandidateTasks = j.contains("candidateTasks") && j["candidateTasks"].is_array();

    // 缓存 agvStatusList（日志节流由 LOG_LEVEL/LOG_STATUS_INTERVAL_SEC 控制）
    if (j.contains("agvStatusList") && j["agvStatusList"].is_array()) {
        cache_agv_status_list(j, robotRepo);
    }

    if (!hasCandidateTasks) {
        std::cout << "[Info] message without candidateTasks: caches updated, skip scheduling." << std::endl;
        return;
    }

    std::shared_lock<std::shared_mutex> mapReadLock(mapMutex);
    const bool mapReadyNow = mapReady.load();
    if (!mapReadyNow || !aStarPtr) {
        std::cout << "[Map] 尚未加载地图，跳过当前调度请求。" << std::endl;
        return;
    }

    // 3) Parse candidateTasks → internal Task list (with sorting/batching)
    std::vector<Task> taskList;
    std::vector<Task> originalTaskList;
    std::vector<int> filteredToOriginal;
    std::vector<int> originalToFiltered;
    std::vector<int> forcedTaskIndices;
    std::vector<std::string> forcedReasons;
    int timeoutMs = 1500;
    if (const char* ev = std::getenv("ALLOC_TIMEOUT_MS")) {
        try { timeoutMs = std::max(1, std::stoi(ev)); } catch (...) {}
    }
    try { timeoutMs = j.value("timeoutMs", timeoutMs); } catch (...) {}
    int maxBatchSize = parse_batch_config(j);
    int maxBatchCap = getenv_int("ALLOC_MAX_BATCH_SIZE", 0);
    if (maxBatchCap > 0) {
        if (maxBatchSize <= 0) maxBatchSize = maxBatchCap;
        else maxBatchSize = std::min(maxBatchSize, maxBatchCap);
    }
    if (j.contains("candidateTasks") && j["candidateTasks"].is_array()) {
        struct TmpTask { Task t; int pri; std::string start; };
        std::vector<TmpTask> tmp;
        const auto& cts = j["candidateTasks"];
        for (const auto& ct : cts) {
            std::string tid = read_string(ct, "taskId", "");
            int pri = 0;
            if (ct.contains("priority")) {
                pri = read_int(ct, "priority", 0);
            } else {
                pri = read_int(ct, "Priority", 0);
            }
            int minBat = read_int(ct, "minBatteryLevel", 0);
            int taskTypeNum = -1;
            if (ct.contains("taskType")) {
                if (ct["taskType"].is_number_integer()) {
                    taskTypeNum = ct["taskType"].get<int>();
                } else if (ct["taskType"].is_string()) {
                    std::string u = read_string(ct, "taskType", "");
                    for (auto& ch : u) ch = ::toupper(ch);
                    if (u == "CHARGE" || u == "CHARGING") taskTypeNum = 3;
                }
            }
            std::string bindRobotId = read_string(ct, "bindRobotId", "");
            std::vector<std::string> deviceIds;
            if (!bindRobotId.empty()) {
                deviceIds.push_back(bindRobotId);
            } else {
                collect_vehicle_filters(ct, deviceIds);
            }
            std::vector<SubTask> sts;
            if (ct.contains("subTasks") && ct["subTasks"].is_array() && !ct["subTasks"].empty()) {
                const auto& subs = ct["subTasks"];
                for (size_t si = 0; si < subs.size(); ++si) {
                    const auto& st = subs[si];
                    int seq = read_int(st, "sequence", static_cast<int>(si + 1));
                    int ptype = parse_point_type(st);
                    std::string loc = read_string(st, "location", "0");
                    json pj = st.contains("point") && st["point"].is_object() ? st["point"] : json::object();
                    int px = read_int(pj, "x", 0), py = read_int(pj, "y", 0), pa = read_int(pj, "angle", 0);
                    int pnode = read_int(pj, "nodeId", -1);
                    Point p(px, py, pa, pnode, ptype);
                    std::int64_t estMs = 0;
                    try {
                        estMs = st.contains("estimatedDuration") ? st["estimatedDuration"].get<std::int64_t>() : 0;
                    } catch (...) {
                        double tmpDur = read_double(st, "estimatedDuration", 0.0);
                        estMs = static_cast<std::int64_t>(std::llround(tmpDur));
                    }
                    std::vector<std::string> pcs;
                    if (st.contains("pathConstraints") && st["pathConstraints"].is_array()) {
                        for (const auto& pc : st["pathConstraints"]) {
                            try { pcs.push_back(pc.get<std::string>()); } catch (...) {}
                        }
                    }
                    int maxSp = read_int(st, "maxSpeed", 0);
                    std::string subTaskId = read_string(st, "subTaskId", "");
                    if (subTaskId.empty() && !tid.empty()) {
                        subTaskId = tid + "#" + std::to_string(seq);
                    }
                    sts.emplace_back(seq, ptype, loc, p, estMs, pcs, maxSp, subTaskId);
                }
            }
            std::string startIso = read_string(ct, "expectedStartTime", "");
            std::string endIso = read_string(ct, "expectedCompletionTime", "");
            std::string tsIso = read_string(ct, "timestamp", "");
            int taskMapId = read_task_map_id(ct, req.mapId);
            Task t(pri, taskTypeNum, 0, tid, deviceIds, timeoutMs, sts, false, taskMapId,
                   minBat, startIso, endIso, tsIso);
            std::string sortKey = startIso.empty() ? std::string("9999-12-31T23:59:59Z") : startIso;
            tmp.push_back(TmpTask{std::move(t), pri, sortKey});
        }
        std::sort(tmp.begin(), tmp.end(), [](const TmpTask& a, const TmpTask& b) {
            if (a.pri != b.pri) return a.pri > b.pri;
            return a.start < b.start;
        });
        size_t takeN = tmp.size();
        if (maxBatchSize > 0) takeN = std::min(tmp.size(), static_cast<size_t>(maxBatchSize));
        taskList.reserve(takeN);
        for (size_t i = 0; i < takeN; ++i) {
            taskList.emplace_back(std::move(tmp[i].t));
        }
    }

    originalTaskList = taskList;
    std::unordered_map<std::string, int> taskPriorityById;
    taskPriorityById.reserve(originalTaskList.size());
    for (const auto& tk : originalTaskList) {
        if (!tk.getMessageId().empty()) {
            taskPriorityById[tk.getMessageId()] = tk.getPriority();
        }
    }

    // 4) Build AMR list from agvStatusList（缺失时记录警告，不再使用静态回退）
    std::vector<Amr> amrList;
    bool missingAgvStatus = false;
    auto statusSnapshot = robotRepo.snapshotStatuses();
    const bool logStatusSnapshot = env_enabled("RECEIVER_LOG_STATUS_SNAPSHOT");
    if (logStatusSnapshot && !statusSnapshot.empty()) {
        json statusDump = json::array();
        for (const auto& st : statusSnapshot) {
            json obj;
            obj["deviceId"] = st.deviceId;
            obj["mapId"] = st.mapId;
            obj["x"] = st.x;
            obj["y"] = st.y;
            obj["angle"] = st.angle;
            obj["nodeId"] = st.nodeId;
            obj["nextDestinationPoint"] = st.nextDestination;
            obj["taskId"] = st.taskId;
            obj["taskStatus"] = st.taskStatus;
            obj["taskProgress"] = st.taskProgress;
            obj["batteryLevel"] = st.batteryLevel;
            obj["updateTime"] = st.updateTime;
            statusDump.push_back(std::move(obj));
        }
        std::cout << "[Debug] robot status snapshot before allocation:\n"
                  << statusDump.dump(2) << std::endl;
    }
    if (!statusSnapshot.empty()) {
        for (const auto& st : statusSnapshot) {
            amrList.emplace_back(build_amr_from_status(st, resolvedAheadMs, resolvedDetourMm, mapInfo));
        }
    } else {
        missingAgvStatus = true;
        std::cout << "[警告] 未缓存到任何 AGV 状态，本次调度将直接输出未分配结果。" << std::endl;
    }

    auto amrsForFilter = TaskPolicy::selectAvailableAmrs(amrList, taskList, mapInfo);
    auto reachability = TaskReachabilityFilter::FilterTasksByReachability(taskList, mapInfo, amrsForFilter);
    taskList = reachability.filteredTasks;
    filteredToOriginal = reachability.filteredToOriginal;
    originalToFiltered = reachability.originalToFiltered;
    forcedTaskIndices = reachability.forcedTaskIndices;
    forcedReasons = reachability.forcedReasons;
    if (!forcedTaskIndices.empty()) {
        std::cout << "[预筛] 有 " << forcedTaskIndices.size()
                  << " 个任务因不可达被标记为未分配" << std::endl;
    }

    auto availableAmrs = TaskPolicy::selectAvailableAmrs(amrList, taskList, mapInfo);
    bool doPlan = TaskPolicy::shouldPlan(amrList, taskList);
    if (env_enabled("RECEIVER_LOG_ALLOC_SUMMARY")) {
        int idleCnt = 0;
        int unavailableCnt = 0;
        int rejectingCnt = 0;
        int workingCnt = 0;
        int speedLe80Cnt = 0;
        for (const auto& amr : amrList) {
            if (amr.isIdle()) idleCnt += 1;
            if (amr.isUnavailable()) unavailableCnt += 1;
            if (amr.isRejectingTasks()) rejectingCnt += 1;
            if (amr.isWorking()) workingCnt += 1;
            if (amr.getSpeed() <= 80) speedLe80Cnt += 1;
        }
        std::cout << "[AllocSummary] amrs=" << amrList.size()
                  << " available=" << availableAmrs.size()
                  << " tasks=" << taskList.size()
                  << " doPlan=" << (doPlan ? 1 : 0)
                  << " idle=" << idleCnt
                  << " working=" << workingCnt
                  << " unavailable=" << unavailableCnt
                  << " rejecting=" << rejectingCnt
                  << " speedLe80=" << speedLe80Cnt
                  << std::endl;
    }
    if (!mapReadyNow) {
        std::cerr << "[Map] 尚未接收到有效地图，忽略调度请求。" << std::endl;
        return;
    }
    bool plannerReady = (aStarPtr != nullptr) && (staticTableReady || skipStaticTable);
    if (!plannerReady) {
        std::cerr << "[Warn] 路径规划器未就绪（静态表或A*无效），无法执行调度。" << std::endl;
        return;
    }
    if (!doPlan || availableAmrs.empty() || taskList.empty()) {
        if (logAllocSummary) {
            std::cout << "\n[提示] 条件不足，不进行任务分配："
                      << " doPlan=" << doPlan
                      << " availableAmrs=" << availableAmrs.size()
                      << " tasks=" << taskList.size() << std::endl;
        }
        if (missingAgvStatus) {
            if (logAllocSummary) {
                std::cout << "[提示] 由于缺少 agvStatusList，AMR 列表为空，请检查发送端是否携带车辆状态。" << std::endl;
            }
        }
        AllocationResult finalResult;
        finalResult.amrTasks.assign(amrList.size(), {});
        finalResult.totalCost = 0.0;
        finalResult.code.clear();
        finalResult.unallocatedTaskIds = forcedTaskIndices;
        for (int origIdx : filteredToOriginal) {
            if (origIdx >= 0) finalResult.unallocatedTaskIds.push_back(origIdx);
        }
        std::sort(finalResult.unallocatedTaskIds.begin(), finalResult.unallocatedTaskIds.end());
        finalResult.unallocatedTaskIds.erase(
            std::unique(finalResult.unallocatedTaskIds.begin(), finalResult.unallocatedTaskIds.end()),
            finalResult.unallocatedTaskIds.end());

        if (!finalResult.unallocatedTaskIds.empty()) {
            if (logAllocSummary) {
                std::cout << "未分配任务数: " << finalResult.unallocatedTaskIds.size() << std::endl;
            }
            int printUnalloc = 0;
            if (const char* ev = std::getenv("EXT_PRINT_UNALLOC_CAUSE")) {
                try { printUnalloc = std::stoi(ev); } catch (...) { printUnalloc = 1; }
            }
            if (printUnalloc) {
                for (int origIdx : finalResult.unallocatedTaskIds) {
                    if (origIdx < 0 || origIdx >= static_cast<int>(originalTaskList.size())) continue;
                    const Task& tk = originalTaskList[origIdx];
                    int filteredIdx = (origIdx >= 0 && origIdx < static_cast<int>(originalToFiltered.size()))
                        ? originalToFiltered[origIdx] : -1;
                    std::string reason;
                    if (filteredIdx == -1) {
                        if (origIdx >= 0 && origIdx < static_cast<int>(forcedReasons.size()) && !forcedReasons[origIdx].empty()) {
                            reason = forcedReasons[origIdx];
                        } else {
                            reason = "静态预筛不可达";
                        }
                    } else {
                        reason = "调度未执行（无可用AMR或条件不足）";
                    }
                    std::cout << "  任务" << tk.getMessageId() << ": " << reason << std::endl;
                }
            }
        }

        long calc_ms = 0;
        double opt_score = 0.0;
        std::vector<std::string> constraints;
        try {
            if (logAllocProfile) {
                std::cout << "[Profiling] publish(empty-conditions) 开始..." << std::endl;
            }
            auto plansOpt = resultPublisher.publish(finalResult, amrList, originalTaskList,
                mapReadyNow ? &mapInfo : nullptr, calc_ms, opt_score, constraints, &req,
                (mapReadyNow && staticTableReady) ? &staticTable : nullptr);
            if (logAllocProfile) {
                std::cout << "[Profiling] publish(empty-conditions) 结束" << std::endl;
            }
	            if (plansOpt.has_value()) {
	                if (logAllocSummary) {
	                    std::cout << "[OK] 结果已发送" << std::endl;
	                }
	                if (mapReadyNow) {
	                     std::string genTime = iso8601_utc_now();
	                     const int keepTtlMs = getenv_int("RESERVE_TTL_MS", 15000);
	                     const int committedTtlMs = std::max(0, getenv_int("RESERVE_COMMITTED_TTL_MS", 0));
		                     for (const auto& plan : plansOpt.value()) {
		                         int keepNodeId = -1;
		                         int keepNextNodeId = -1;
		                         if (!plan.amrId.empty()) {
		                             auto stOpt = robotRepo.getStatusById(plan.amrId);
		                             if (stOpt.has_value()) {
		                                 keepNodeId = resolve_status_node_id(*stOpt, mapInfo);
		                                 keepNextNodeId = resolve_committed_next_node_id(*stOpt, mapInfo);
		                             }
		                         }
		                         if (!plan.amrId.empty() && keepNodeId >= 0) {
		                             std::vector<int> committed = robotRepo.getLastSentRoute(plan.amrId);
		                             std::unordered_set<int> committedSet(committed.begin(), committed.end());
		                             std::vector<int> keepNodes;
		                             keepNodes.push_back(keepNodeId);
		                             if (keepNextNodeId >= 0 && keepNextNodeId != keepNodeId) {
		                                 keepNodes.push_back(keepNextNodeId);
		                             }
		                             for (int nodeId : committed) {
		                                 if (std::find(keepNodes.begin(), keepNodes.end(), nodeId) == keepNodes.end()) {
		                                     keepNodes.push_back(nodeId);
		                                 }
		                             }
		                             nodeReservations.releaseAllByOwnerExceptSet(plan.amrId, keepNodes);
		                             int curTtlMs = 0;
		                             nodeReservations.tryReserve(
		                                 keepNodeId,
		                                 plan.amrId,
		                                 NodeReservationTable::HoldReason::WAITING_POINT,
		                                 "plan_update",
		                                 std::chrono::milliseconds(curTtlMs));
		                             if (keepNextNodeId >= 0 && keepNextNodeId != keepNodeId) {
		                                 int nextTtlMs = keepTtlMs;
		                                 if (committedSet.count(keepNextNodeId) > 0) {
		                                     nextTtlMs = committedTtlMs > 0 ? committedTtlMs : 0;
		                                 }
		                                 nodeReservations.tryReserve(
		                                     keepNextNodeId,
		                                     plan.amrId,
		                                     NodeReservationTable::HoldReason::RESERVED_PATH,
		                                     "committed_edge",
		                                     std::chrono::milliseconds(std::max(0, nextTtlMs)));
		                             }
	                } else if (!plan.amrId.empty()) {
	                    nodeReservations.releaseAllByOwner(plan.amrId);
	                }
	                robotRepo.updatePlan(plan.amrId, plan, req.messageId, genTime, taskPriorityById);
	            }
	        }
	            } else {
                if (logAllocSummary) {
                    std::cout << "[WARN] 结果发送失败" << std::endl;
                }
            }
        } catch (...) {
            if (logAllocSummary) {
                std::cout << "[WARN] 结果发送异常" << std::endl;
            }
        }
	        if (mapReadyNow && staticTableReady && aStarPtr) {
	            auto plans = PathPlanningHelper::BuildAmrPathPlans(
	                finalResult, amrList, originalTaskList, staticTable, *aStarPtr, mapInfo);
	            std::string genTime = iso8601_utc_now();
		            const int keepTtlMs = getenv_int("RESERVE_TTL_MS", 15000);
		            const int committedTtlMs = std::max(0, getenv_int("RESERVE_COMMITTED_TTL_MS", 0));
		            for (const auto& plan : plans) {
		                int keepNodeId = -1;
		                int keepNextNodeId = -1;
		                if (!plan.amrId.empty()) {
		                    auto stOpt = robotRepo.getStatusById(plan.amrId);
		                    if (stOpt.has_value()) {
		                        keepNodeId = resolve_status_node_id(*stOpt, mapInfo);
		                        keepNextNodeId = resolve_committed_next_node_id(*stOpt, mapInfo);
		                    }
		                }
		                if (!plan.amrId.empty() && keepNodeId >= 0) {
		                    std::vector<int> committed = robotRepo.getLastSentRoute(plan.amrId);
		                    std::unordered_set<int> committedSet(committed.begin(), committed.end());
		                    std::vector<int> keepNodes;
		                    keepNodes.push_back(keepNodeId);
		                    if (keepNextNodeId >= 0 && keepNextNodeId != keepNodeId) {
		                        keepNodes.push_back(keepNextNodeId);
		                    }
		                    for (int nodeId : committed) {
		                        if (std::find(keepNodes.begin(), keepNodes.end(), nodeId) == keepNodes.end()) {
		                            keepNodes.push_back(nodeId);
		                        }
		                    }
		                    nodeReservations.releaseAllByOwnerExceptSet(plan.amrId, keepNodes);
		                    int curTtlMs = 0;
		                    nodeReservations.tryReserve(
		                        keepNodeId,
		                        plan.amrId,
		                        NodeReservationTable::HoldReason::WAITING_POINT,
		                        "plan_update",
		                        std::chrono::milliseconds(curTtlMs));
		                    if (keepNextNodeId >= 0 && keepNextNodeId != keepNodeId) {
		                        int nextTtlMs = keepTtlMs;
		                        if (committedSet.count(keepNextNodeId) > 0) {
		                            nextTtlMs = committedTtlMs > 0 ? committedTtlMs : 0;
		                        }
		                        nodeReservations.tryReserve(
		                            keepNextNodeId,
		                            plan.amrId,
		                            NodeReservationTable::HoldReason::RESERVED_PATH,
		                            "committed_edge",
		                            std::chrono::milliseconds(std::max(0, nextTtlMs)));
		                    }
		                } else if (!plan.amrId.empty()) {
	                    nodeReservations.releaseAllByOwner(plan.amrId);
	                }
	                robotRepo.updatePlan(plan.amrId, plan, req.messageId, genTime, taskPriorityById);
	            }
	        }
        ordered_json trafficPayload =
            build_traffic_path_payload(req.messageId, mapId.load(), robotRepo, mapInfo, taskPriorityById,
                                       mapReadyNow, staticTableReady, skipStaticTable, staticTable,
                                       aStarPtr.get(), nodeReservations);
        std::string trafficPayloadStr = trafficPayload.dump();
        trafficPathPublisher.publishPayload(trafficPayloadStr);
        trafficDebug.onPublish(trafficPayload, trafficPayloadStr, "allocation");
        if (trail_send_on_allocation(g_trailVersion)) {
            publish_trail_for_all_plans(
                req.messageId,
                robotRepo,
                mapInfo,
                mapMutex,
                mapReadyNow,
                skipStaticTable,
                staticTableReady,
                staticTable,
                aStarPtr.get(),
                nodeReservations,
                algoPublisher,
                false,
                false);
        }
        std::string runDesc = "assigned=" +
            std::to_string(count_assigned_tasks(finalResult)) +
            " unassigned=" + std::to_string(finalResult.unallocatedTaskIds.size());
        algoPublisher.sendRunInfo(serviceName, runDesc, format_local_time());
        if (!finalResult.unallocatedTaskIds.empty()) {
            std::string alertDesc = "Unassigned tasks: " + std::to_string(finalResult.unallocatedTaskIds.size());
            algoPublisher.sendAlertInfo(serviceName, 1001, "B", alertDesc, format_local_time());
        }
        return;
    }

    if (TaskPolicy::hasEmergency(taskList) && !TaskPolicy::hasIdleAmr(amrList)) {
        for (auto& a : amrList) {
            if (a.isMoving()) {
                auto stolen = a.stealAllQueuedTasks();
                taskList.insert(taskList.end(), stolen.begin(), stolen.end());
            } else if (a.isWorking()) {
                auto stolen = a.stealQueuedTasksExceptFirst();
                taskList.insert(taskList.end(), stolen.begin(), stolen.end());
            }
        }
    }
    std::vector<int> startNodeOverride(availableAmrs.size(), -1);
    std::vector<double> startTimeOffsetMs(availableAmrs.size(), 0.0);
    if (TaskPolicy::hasEmergency(taskList) && !TaskPolicy::hasIdleAmr(amrList)) {
        int workerNumTmp = 2;
        if (const char* ev = std::getenv("ALLOC_WORKERS")) {
            try { workerNumTmp = std::max(1, std::stoi(ev)); } catch (...) {}
        }
        ShortestPathUpdater tmpUpdater(
            mapInfo,
            PathPlanningConstants::kDefaultTurnPenaltyMm,
            workerNumTmp
        );
        int maxTypeTmp = 0;
        for (const auto& a : amrList) maxTypeTmp = std::max(maxTypeTmp, a.getDeviceType());
        tmpUpdater.initializeMatrix(std::max(1, maxTypeTmp + 1));
        for (size_t i = 0; i < availableAmrs.size(); ++i) {
            const Amr& a = availableAmrs[i];
            if (a.isMoving()) {
                if (a.getNextDestinationPointId() != -1) startNodeOverride[i] = a.getNextDestinationPointId();
                else startNodeOverride[i] = mapInfo.findNearestNodeId(a.getX(), a.getY());
                startTimeOffsetMs[i] = 0.0;
            } else if (a.isWorking()) {
                int predictedEnd = -1;
                const auto& trail = a.getCurTrailPoints();
                if (!trail.empty()) predictedEnd = trail.back().getNodeId();
                else if (a.getNextDestinationPointId() != -1) predictedEnd = a.getNextDestinationPointId();
                else predictedEnd = mapInfo.findNearestNodeId(a.getX(), a.getY());
                startNodeOverride[i] = predictedEnd;
                int curNodeId = mapInfo.findNearestNodeId(a.getX(), a.getY());
                int amrTypeIndex = a.getDeviceType();
                double offsetSec = tmpUpdater.queryByNodeId(curNodeId, predictedEnd, amrTypeIndex);
                startTimeOffsetMs[i] = (offsetSec > 0) ? offsetSec * 1000.0 : 0.0;
            } else {
                startTimeOffsetMs[i] = 0.0;
            }
        }
    }

    auto pickTaskEndpoints = [&](const Task& tk) -> std::pair<int, int> {
        int sId = tk.getStartId();
        int eId = tk.getEndId();
        const auto& subs = tk.getSubTasks();
        if (!subs.empty()) {
            sId = subs.front().getPoint().getNodeId();
            eId = subs.back().getPoint().getNodeId();
        }
        return {sId, eId};
    };

    int workerNum = 4;
    if (const char* ev = std::getenv("ALLOC_WORKERS")) {
        try { workerNum = std::max(1, std::stoi(ev)); } catch (...) {}
    }
    ShortestPathUpdater pathUpdater(
        mapInfo,
        PathPlanningConstants::kDefaultTurnPenaltyMm,
        workerNum
    );
    int maxTypeIdx = 0;
    for (const auto& a : availableAmrs) maxTypeIdx = std::max(maxTypeIdx, a.getDeviceType());
    pathUpdater.initializeMatrix(std::max(1, maxTypeIdx + 1));
    std::string algo = req.allocationAlgorithm.empty() ? "posta" : req.allocationAlgorithm;
    if (algo != "posta" && algo != "greedy" && algo != "mlp") {
        if (logAllocSummary) {
            std::cout << "[Warn] 未识别算法 '" << algo << "'，默认使用 posta" << std::endl;
        }
        algo = "posta";
    }
#if !HAS_MLP_ALLOCATOR
    if (algo == "mlp") {
        if (logAllocSummary) {
            std::cout << "[Warn] 未编译 MLP 分配器，回退到 POSTA" << std::endl;
        }
        algo = "posta";
    }
#endif
    req.allocationAlgorithm = algo;
    if (logAllocSummary) {
        std::cout << "[Alloc] algorithm=" << algo << std::endl;
    }

    if (algo != "mlp") {
        auto tCostBeg = std::chrono::steady_clock::now();
        if (logAllocProfile) {
            std::cout << "[Profiling] 开始生成成本矩阵，amr=" << availableAmrs.size()
                      << " tasks=" << taskList.size()
                      << " workers=" << workerNum
                      << (skipStaticTable ? " (纯A*)" : " (混合规划)") << std::endl;
        }
        auto costMatrix = CostMatrixGenerator::generateCostMatrix(availableAmrs, taskList, pathUpdater, mapInfo, startNodeOverride, startTimeOffsetMs);
        auto tCostEnd = std::chrono::steady_clock::now();
        if (logAllocProfile) {
            std::cout << "[Profiling] 成本矩阵生成完成，耗时 "
                      << std::chrono::duration<double, std::milli>(tCostEnd - tCostBeg).count()
                      << " ms" << std::endl;
        }
        (void)costMatrix;
    } else {
        if (logAllocProfile) {
            std::cout << "[Profiling] 跳过成本矩阵生成（mlp）" << std::endl;
        }
        TaskAllocationUtils::sparseReset(static_cast<int>(taskList.size()), static_cast<int>(availableAmrs.size()));
        double globalSpeed = mapInfo.getGlobalMaxSpeed();
        if (!(globalSpeed > 0.0)) globalSpeed = 1000.0;
        TaskAllocationUtils::setGlobalSpeedMmPerSec(globalSpeed);
        TaskAllocationUtils::setMapInfoPtr(&mapInfo);
    }

    bool diag = false;
    if (const char* ev = std::getenv("ALLOC_DIAG")) {
        std::string v(ev);
        diag = !(v == "0" || v == "false" || v == "FALSE");
    }
    double budgetSec = std::max(0.01, timeoutMs / 1000.0);

    auto tAlgoBeg = std::chrono::steady_clock::now();
    GreedyTaskAllocator greedy;
    AllocationResult greedyRes;
    bool greedyReady = false;
    auto tAfterGreedy = tAlgoBeg;
    double greedyElapsed = 0.0;
    auto run_greedy = [&]() {
        if (greedyReady) return;
        if (logAllocProfile) {
            std::cout << "[Profiling] Greedy 分配开始..." << std::endl;
        }
        greedyRes = greedy.allocate(availableAmrs, taskList);
        tAfterGreedy = std::chrono::steady_clock::now();
        greedyElapsed = std::chrono::duration<double>(tAfterGreedy - tAlgoBeg).count();
        if (logAllocProfile) {
            std::cout << "[Profiling] Greedy 分配完成，耗时 "
                      << std::chrono::duration<double, std::milli>(tAfterGreedy - tAlgoBeg).count()
                      << " ms" << std::endl;
        }
        greedyReady = true;
    };
    auto read_env_int = [](const char* k, int d) {
        if (const char* v = std::getenv(k)) {
            try { return std::stoi(v); } catch (...) {}
        }
        return d;
    };
    auto read_env_double = [](const char* k, double d) {
        if (const char* v = std::getenv(k)) {
            try { return std::stod(v); } catch (...) {}
        }
        return d;
    };
    int postaSE = read_env_int("ALLOC_POSTA_SE", 30);
    int postaSweeps = read_env_int("ALLOC_POSTA_SWEEPS", 10);
    double priCoeff = read_env_double("ALLOC_PRI_COEFF", 0.02);
    double topkRatio = read_env_double("ALLOC_TOPK_RATIO", 0.5);
    int topk = read_env_int("ALLOC_TOPK", -1);
    int topkAgent = read_env_int("ALLOC_TOPK_AGENT", (topk > 0 ? topk : -1));
    int topkNext = read_env_int("ALLOC_TOPK_NEXT", (topk > 0 ? topk : -1));
    auto run_posta = [&]() {
        run_greedy();
        double remainSec = std::max(0.01, budgetSec - greedyElapsed - 0.01);
        if (logAllocProfile) {
            std::cout << "[POSTA Params] SE=" << postaSE
                      << " sweeps=" << postaSweeps
                      << " pri_coeff=" << priCoeff
                      << " | workers=" << workerNum
                      << " timeoutSec(remain)=" << remainSec
                      << " | topk_ratio=" << topkRatio
                      << " topk_agent=" << (topkAgent > 0 ? std::to_string(topkAgent) : std::string("(auto)"))
                      << " topk_next=" << (topkNext > 0 ? std::to_string(topkNext) : std::string("(auto)"))
                      << std::endl;
        }
        PostaTaskAllocator posta;
        if (!std::getenv("ALLOC_SEED")) {
            uint32_t seedFromReq = 0;
            if (!req.messageId.empty()) {
                seedFromReq = static_cast<uint32_t>(std::hash<std::string>{}(req.messageId));
            }
            if (seedFromReq == 0) {
                seedFromReq = static_cast<uint32_t>(
                    std::chrono::steady_clock::now().time_since_epoch().count());
            }
            posta.setRandomSeed(seedFromReq);
        }
        posta.setCandidateCount(30);
        posta.setBanShuffle(false);
        posta.setTimeLimit(remainSec);
        posta.setPriorityPenaltyCoeff(priCoeff);
        posta.setPriorityPenaltyCurve({0.0, 0.4, 0.5, 0.7, 0.9, 1.2, 1.6, 2.1, 2.7, 3.4, 4.2});
        posta.setInitialSolution(greedyRes.code);
        if (logAllocProfile) {
            std::cout << "[Profiling] POSTA 分配开始..." << std::endl;
        }
        auto postaRes = posta.allocate(availableAmrs, taskList);
        auto tPostaEnd = std::chrono::steady_clock::now();
        if (logAllocProfile) {
            std::cout << "[Profiling] POSTA 分配完成，耗时 "
                      << std::chrono::duration<double, std::milli>(tPostaEnd - tAfterGreedy).count()
                      << " ms" << std::endl;
        }
        return postaRes;
    };

    AllocationResult res;
    if (algo == "greedy") {
        run_greedy();
        res = greedyRes;
    } else if (algo == "mlp") {
#if HAS_MLP_ALLOCATOR
        if (logAllocProfile) {
            std::cout << "[Profiling] MLP 分配开始..." << std::endl;
        }
        MlpTaskAllocator mlp;
        res = mlp.allocate(availableAmrs, taskList);
        auto tMlpEnd = std::chrono::steady_clock::now();
        if (logAllocProfile) {
            std::cout << "[Profiling] MLP 分配完成，耗时 "
                      << std::chrono::duration<double, std::milli>(tMlpEnd - tAlgoBeg).count()
                      << " ms" << std::endl;
        }
#else
        if (logAllocSummary) {
            std::cout << "[Warn] 未编译 MLP 分配器，回退到 POSTA" << std::endl;
        }
        algo = "posta";
        req.allocationAlgorithm = algo;
        res = run_posta();
#endif
    } else {
        res = run_posta();
    }
    auto tSanitizeBeg = std::chrono::steady_clock::now();
    if (logAllocProfile) {
        std::cout << "[Profiling] 准备进入分配后可达性修正..." << std::endl;
        std::cout << "[Profiling] 分配后可达性修正开始..." << std::endl;
    }
    res = SanitizeAllocationResult(res, availableAmrs, taskList, mapInfo, pathUpdater);
    auto tSanitizeEnd = std::chrono::steady_clock::now();
    if (logAllocProfile) {
        std::cout << "[Profiling] 分配后可达性修正完成，耗时 "
                  << std::chrono::duration<double, std::milli>(tSanitizeEnd - tSanitizeBeg).count()
                  << " ms" << std::endl;
    }
    if (diag) {
        if (logAllocDiag) {
            if (!res.repairLog.empty()) {
                std::cout << "\n== Diagnostics ==\n" << res.repairLog << "\n";
            } else {
                std::cout << "\n== Diagnostics ==\n所有任务路径可达，无需额外修复\n";
            }
        }
    }
    auto tAlgoEnd = std::chrono::steady_clock::now();
    double elapsedSec = std::chrono::duration<double>(tAlgoEnd - tAlgoBeg).count();

    AllocationResult mappedRes = TaskReachabilityFilter::RemapAllocationToOriginal(
        res, filteredToOriginal, originalTaskList.size());
    auto parsedTasks = mappedRes.amrTasks.empty()
        ? TaskAllocationUtils::parseSolution(mappedRes.code)
        : mappedRes.amrTasks;
    std::vector<int> availableToFull(availableAmrs.size(), -1);
    std::vector<std::vector<int>> fullAmrTasks(amrList.size());
    for (size_t i = 0; i < availableAmrs.size(); ++i) {
        const std::string& did = availableAmrs[i].getDeviceId();
        int fullIdx = -1;
        for (size_t j2 = 0; j2 < amrList.size(); ++j2) {
            if (amrList[j2].getDeviceId() == did) {
                fullIdx = static_cast<int>(j2);
                break;
            }
        }
        availableToFull[i] = fullIdx;
        if (fullIdx >= 0 && fullIdx < static_cast<int>(fullAmrTasks.size()) && i < parsedTasks.size()) {
            fullAmrTasks[fullIdx] = parsedTasks[i];
        }
    }
    mappedRes.amrTasks = std::move(fullAmrTasks);
    mappedRes.code = TaskAllocationUtils::deParseSolution(mappedRes.amrTasks);

    auto remainingForced = TaskReachabilityFilter::InsertForcedTasks(
        mappedRes, forcedTaskIndices, availableToFull, amrList, originalTaskList, pathUpdater, mapInfo);
    TaskReachabilityFilter::AppendForcedTasks(mappedRes, remainingForced);

    auto finalDurations = TaskReachabilityFilter::ComputeChainDurationsSec(
        mappedRes, amrList, originalTaskList, pathUpdater, mapInfo);
    double totalCostSec = 0.0;
    double makespanSec = 0.0;
    // 若链路不可达（INF），将该车的任务全部标为未分配，避免总成本为 INF
    std::vector<int> unreachableTasks;
    for (size_t i = 0; i < finalDurations.size() && i < mappedRes.amrTasks.size(); ++i) {
        if (std::isfinite(finalDurations[i])) continue;
        for (int tidx : mappedRes.amrTasks[i]) {
            if (tidx >= 0) unreachableTasks.push_back(tidx);
        }
        mappedRes.amrTasks[i].clear();
    }
    if (!unreachableTasks.empty()) {
        for (int tidx : unreachableTasks) mappedRes.unallocatedTaskIds.push_back(tidx);
        std::sort(mappedRes.unallocatedTaskIds.begin(), mappedRes.unallocatedTaskIds.end());
        mappedRes.unallocatedTaskIds.erase(std::unique(mappedRes.unallocatedTaskIds.begin(), mappedRes.unallocatedTaskIds.end()),
                                           mappedRes.unallocatedTaskIds.end());
        mappedRes.code = TaskAllocationUtils::deParseSolution(mappedRes.amrTasks);
        finalDurations = TaskReachabilityFilter::ComputeChainDurationsSec(
            mappedRes, amrList, originalTaskList, pathUpdater, mapInfo);
    }
    for (double d : finalDurations) {
        if (!std::isfinite(d)) {
            totalCostSec = std::numeric_limits<double>::infinity();
            makespanSec = std::numeric_limits<double>::infinity();
            break;
        }
        totalCostSec += d;
        makespanSec = std::max(makespanSec, d);
    }
    mappedRes.totalCost = totalCostSec;
    AllocationResult finalResult = mappedRes;

    if (logAllocFullResult) {
        std::cout << "\n-- External Receiver Allocation Result --\n";
        if (elapsedSec > budgetSec - 1e-3) {
            std::cout << "[分配失败] 超时：elapsed=" << elapsedSec << "s > timeoutMs=" << (budgetSec * 1000.0) << "ms" << std::endl;
        }
        if (std::isfinite(totalCostSec)) std::cout << "总成本: " << totalCostSec << " s" << std::endl;
        else std::cout << "总成本: inf" << std::endl;
        for (size_t i = 0; i < amrList.size(); ++i) {
            const Amr& amr = amrList[i];
            int posNode = AmrPositionResolver::resolveStartNodeId(amr, mapInfo);
            std::string agvId = amr.getDeviceId();
            if (agvId.empty()) agvId = "amr" + std::to_string(i + 1);
            std::cout << "agvId=" << agvId << "(bat:" << amr.getBatteryLevel()
                      << ", position:" << posNode << ") : ";
            const auto& chain = finalResult.amrTasks.size() > i ? finalResult.amrTasks[i] : std::vector<int>{};
            if (chain.empty()) std::cout << "(无)";
            for (size_t k = 0; k < chain.size(); ++k) {
                int idx0 = chain[k];
                if (idx0 >= 0 && idx0 < static_cast<int>(originalTaskList.size())) {
                    auto [sId, eId] = pickTaskEndpoints(originalTaskList[idx0]);
                    std::cout << originalTaskList[idx0].getMessageId()
                              << "(" << sId << " -> " << eId << ")";
                } else {
                    std::cout << idx0;
                }
                if (k + 1 < chain.size()) std::cout << " - ";
            }
            double chainCost = (i < finalDurations.size()) ? finalDurations[i] : 0.0;
            if (!chain.empty()) {
                if (std::isfinite(chainCost)) std::cout << " （总成本" << chainCost << " s）";
                else std::cout << " （总成本INF）";
            }
            std::cout << std::endl;
        }
        std::cout << "makespan(s): ";
        if (std::isfinite(makespanSec)) std::cout << makespanSec << std::endl;
        else std::cout << "inf" << std::endl;
        if (amrList.size() > availableAmrs.size()) {
            std::cout << "过滤掉的AGV（未参与本次分配）:" << std::endl;
            for (const auto& amr : amrList) {
                bool used = false;
                for (const auto& a2 : availableAmrs) {
                    if (a2.getDeviceId() == amr.getDeviceId()) { used = true; break; }
                }
                if (!used) {
                    std::string reason = "状态/距离等原因未参与";
                    if (amr.isRejectingTasks()) {
                        reason = "taskStatus=不接受任务";
                    } else if (amr.isUnavailable()) {
                        reason = "taskStatus=失败/未执行/暂停（不可用）";
                    } else if (!amr.isIdle()) {
                        reason = "非紧急任务且车辆非空闲";
                    }
                    std::cout << "  [" << amr.getDeviceId()
                              << "] taskStatus=" << amr.getTaskStatus()
                              << "（" << reason << "）" << std::endl;
                }
            }
        }
    }

    if (logAllocSummary) {
        std::unordered_set<int> assignedTaskIdx;
        for (const auto& chain : finalResult.amrTasks) {
            for (int idx : chain) {
                if (idx >= 0) assignedTaskIdx.insert(idx);
            }
        }
        std::cout << "[AllocSummary] tasks=" << originalTaskList.size()
                  << " availableAmrs=" << availableAmrs.size()
                  << " assignedTaskCount=" << assignedTaskIdx.size()
                  << " unallocated=" << finalResult.unallocatedTaskIds.size();
        if (std::isfinite(makespanSec)) {
            std::cout << " makespanSec=" << makespanSec;
        }
        std::cout << std::endl;
    }

    if (logAllocSummary || logAllocFullResult) {
        std::cout << "未分配任务数: " << finalResult.unallocatedTaskIds.size() << std::endl;
    }
    if (!finalResult.unallocatedTaskIds.empty()) {
        if (logAllocSummary || logAllocFullResult) {
            const int printMaxIds = std::max(0, getenv_int("ALLOC_UNALLOC_PRINT_MAX_IDS", 30));
            const size_t showCount = std::min(finalResult.unallocatedTaskIds.size(), static_cast<size_t>(printMaxIds));
            std::cout << "未分配任务(id,前" << showCount << "个): ";
            for (size_t i = 0; i < showCount; ++i) {
                int idx0 = finalResult.unallocatedTaskIds[i];
                if (idx0 >= 0 && idx0 < static_cast<int>(originalTaskList.size())) {
                    std::cout << originalTaskList[idx0].getMessageId();
                } else {
                    std::cout << idx0;
                }
                if (i + 1 < showCount) std::cout << ",";
            }
            if (finalResult.unallocatedTaskIds.size() > showCount) {
                std::cout << " ... +" << (finalResult.unallocatedTaskIds.size() - showCount) << "个";
            }
            std::cout << std::endl;
        }

        int printUnalloc = 0;
        if (const char* ev = std::getenv("EXT_PRINT_UNALLOC_CAUSE")) {
            try { printUnalloc = std::stoi(ev); } catch (...) { printUnalloc = 1; }
        }
        if (printUnalloc) {
            std::cout << "[未分配原因分析]" << std::endl;
            int amrNum2 = static_cast<int>(availableAmrs.size());
            int taskNum2 = static_cast<int>(taskList.size());
            std::vector<double> enduranceSec(amrNum2, 1e18);
            for (int i2 = 0; i2 < amrNum2; ++i2) {
                double e = availableAmrs[i2].getEndurance();
                enduranceSec[i2] = (e > 0 ? e * 3600.0 * 1000.0 : 1e18);
            }
            for (int origIdx : finalResult.unallocatedTaskIds) {
                if (origIdx < 0 || origIdx >= static_cast<int>(originalTaskList.size())) continue;
                int filteredIdx = (origIdx >= 0 && origIdx < static_cast<int>(originalToFiltered.size()))
                    ? originalToFiltered[origIdx] : -1;
                const Task& tkOrig = originalTaskList[origIdx];
                if (filteredIdx == -1) {
                    std::string reason = "静态预筛不可达";
                    if (origIdx >= 0 && origIdx < static_cast<int>(forcedReasons.size()) && !forcedReasons[origIdx].empty()) {
                        reason = forcedReasons[origIdx];
                    }
                    std::cout << " - 任务" << tkOrig.getMessageId() << ": " << reason << std::endl;
                    continue;
                }
                const Task& tk = taskList[filteredIdx];
                bool anyFeasible = false;
                double minInit = std::numeric_limits<double>::infinity();
                for (int k2 = 0; k2 < amrNum2; ++k2) {
                    double c = TaskAllocationUtils::getCost(taskNum2, filteredIdx, k2);
                    if (c < std::numeric_limits<double>::infinity()) {
                        anyFeasible = true;
                        minInit = std::min(minInit, c);
                    }
                }
                std::string rid = tk.getMessageId();
                if (!anyFeasible) {
                    std::cout << " - 任务" << rid << ": 无可执行AMR（类型/设备限定/不可达）" << std::endl;
                    continue;
                }
                bool enduranceHopeless = true;
                for (int k2 = 0; k2 < amrNum2; ++k2) {
                    double c = TaskAllocationUtils::getCost(taskNum2, filteredIdx, k2);
                    if (c < std::numeric_limits<double>::infinity() && c <= enduranceSec[k2]) {
                        enduranceHopeless = false;
                        break;
                    }
                }
                if (enduranceHopeless) {
                    std::cout << " - 任务" << rid << ": 续航不足（初始成本已超过续航上限）" << std::endl;
                } else {
                    std::cout << " - 任务" << rid << ": 无法插入（可能被detour截断/续航叠加/优先级影响）" << std::endl;
                }
            }
            if (!res.repairLog.empty()) {
                std::cout << "[修复日志] " << res.repairLog << std::endl;
            }
        }
    }

    {
        long calc_ms = static_cast<long>(elapsedSec * 1000.0);
        double opt_score = 0.0;
        std::vector<std::string> constraints;
        std::vector<PathPlanningHelper::AmrPlanInfo> plansToCache;
        try {
            if (logAllocProfile) {
                std::cout << "[Profiling] publish(full) 开始..." << std::endl;
            }
            auto plansOpt = resultPublisher.publish(finalResult, amrList, originalTaskList,
                mapReadyNow ? &mapInfo : nullptr, calc_ms, opt_score, constraints, &req,
                (mapReadyNow && staticTableReady) ? &staticTable : nullptr);
            if (logAllocProfile) {
                std::cout << "[Profiling] publish(full) 结束" << std::endl;
            }
            if (plansOpt.has_value()) {
                if (logAllocSummary) {
                    std::cout << "[OK] 结果已发送" << std::endl;
                }
                plansToCache = plansOpt.value();
            } else {
                if (logAllocSummary) {
                    std::cout << "[WARN] 结果发送失败" << std::endl;
                }
            }
        } catch (...) {
            if (logAllocSummary) {
                std::cout << "[WARN] 结果发送异常" << std::endl;
            }
        }

        if (plansToCache.empty() && mapReadyNow && staticTableReady && aStarPtr) {
            plansToCache = PathPlanningHelper::BuildAmrPathPlans(
                finalResult, amrList, originalTaskList, staticTable, *aStarPtr, mapInfo);
        }
	        if (!plansToCache.empty()) {
	            std::string genTime = iso8601_utc_now();
		            const int keepTtlMs = getenv_int("RESERVE_TTL_MS", 15000);
		            const int committedTtlMs = std::max(0, getenv_int("RESERVE_COMMITTED_TTL_MS", 0));
		            for (const auto& plan : plansToCache) {
		                int keepNodeId = -1;
		                int keepNextNodeId = -1;
		                std::optional<RobotStatusEntry> stOpt;
		                if (!plan.amrId.empty()) {
		                    stOpt = robotRepo.getStatusById(plan.amrId);
		                    if (stOpt.has_value()) {
		                        keepNodeId = resolve_status_node_id(*stOpt, mapInfo);
		                        keepNextNodeId = resolve_committed_next_node_id(*stOpt, mapInfo);
		                    }
		                }
		                if (!plan.amrId.empty() && keepNodeId >= 0) {
		                    std::vector<int> committed = robotRepo.getLastSentRoute(plan.amrId);
		                    std::unordered_set<int> committedSet(committed.begin(), committed.end());
		                    std::vector<int> keepNodes;
		                    keepNodes.push_back(keepNodeId);
		                    if (keepNextNodeId >= 0 && keepNextNodeId != keepNodeId) {
		                        keepNodes.push_back(keepNextNodeId);
		                    }
		                    for (int nodeId : committed) {
		                        if (std::find(keepNodes.begin(), keepNodes.end(), nodeId) == keepNodes.end()) {
		                            keepNodes.push_back(nodeId);
		                        }
		                    }
		                    nodeReservations.releaseAllByOwnerExceptSet(plan.amrId, keepNodes);
		                    int curTtlMs = 0;
		                    nodeReservations.tryReserve(
		                        keepNodeId,
		                        plan.amrId,
		                        NodeReservationTable::HoldReason::WAITING_POINT,
		                        "plan_update",
		                        std::chrono::milliseconds(curTtlMs));
		                    if (keepNextNodeId >= 0 && keepNextNodeId != keepNodeId) {
		                        int nextTtlMs = keepTtlMs;
		                        if (committedSet.count(keepNextNodeId) > 0) {
		                            nextTtlMs = committedTtlMs > 0 ? committedTtlMs : 0;
		                        }
		                        nodeReservations.tryReserve(
		                            keepNextNodeId,
		                            plan.amrId,
		                            NodeReservationTable::HoldReason::RESERVED_PATH,
		                            "committed_edge",
		                            std::chrono::milliseconds(std::max(0, nextTtlMs)));
		                    }
		                } else if (!plan.amrId.empty()) {
	                    nodeReservations.releaseAllByOwner(plan.amrId);
	                }
	                robotRepo.updatePlan(plan.amrId, plan, req.messageId, genTime, taskPriorityById);
	                if (!plan.amrId.empty() && keepNodeId >= 0 && aStarPtr) {
	                    auto planEntryOpt = robotRepo.getPlan(plan.amrId);
	                    if (planEntryOpt.has_value()) {
	                        int committedNextNode = -1;
	                        int batteryLevel = 100;
	                        int deviceType = 0;
	                        if (stOpt.has_value()) {
	                            committedNextNode = resolve_committed_next_node_id(*stOpt, mapInfo);
	                            batteryLevel = stOpt->batteryLevel;
	                            deviceType = stOpt->deviceType;
	                        }
	                        const int reserveBudgetOverride = std::max(1, getenv_int("TRAIL_MAX_POINTS", 10));
	                        auto planned = compute_dynamic_plan_to_next_target(
	                            plan.amrId,
	                            keepNodeId,
	                            committedNextNode,
	                            batteryLevel,
	                            deviceType,
	                            *planEntryOpt,
	                            robotRepo,
	                            mapInfo,
	                            skipStaticTable,
	                            staticTableReady,
	                            staticTable,
	                            *aStarPtr,
	                            nodeReservations,
	                            reserveBudgetOverride,
	                            true,
	                            ReplanStage::NONE,
	                            nullptr);
	                        robotRepo.updateDynamicPlan(plan.amrId, planned, planEntryOpt->pathId, reserveBudgetOverride);
	                    }
	                }
	            }
	        }
        ordered_json trafficPayload =
            build_traffic_path_payload(req.messageId, mapId.load(), robotRepo, mapInfo, taskPriorityById,
                                       mapReadyNow, staticTableReady, skipStaticTable, staticTable,
                                       aStarPtr.get(), nodeReservations);
        std::string trafficPayloadStr = trafficPayload.dump();
        trafficPathPublisher.publishPayload(trafficPayloadStr);
        trafficDebug.onPublish(trafficPayload, trafficPayloadStr, "allocation");
        if (trail_send_on_allocation(g_trailVersion)) {
            publish_trail_for_all_plans(
                req.messageId,
                robotRepo,
                mapInfo,
                mapMutex,
                mapReadyNow,
                skipStaticTable,
                staticTableReady,
                staticTable,
                aStarPtr.get(),
                nodeReservations,
                algoPublisher,
                false,
                false);
        }
    }

    {
        const int amrNum2 = static_cast<int>(availableAmrs.size());
        const int taskNum2 = static_cast<int>(taskList.size());
        auto parsed2 = res.amrTasks.empty() ? TaskAllocationUtils::parseSolution(res.code) : res.amrTasks;
        std::vector<int> amrNoTaskIdx;
        for (int i2 = 0; i2 < amrNum2; ++i2) {
            if (i2 < static_cast<int>(parsed2.size()) && parsed2[i2].empty()) amrNoTaskIdx.push_back(i2);
        }
        if (!amrNoTaskIdx.empty()) {
            for (int i2 : amrNoTaskIdx) {
                double best = std::numeric_limits<double>::infinity();
                int bestTask = -1;
                for (int t = 0; t < taskNum2; ++t) {
                    double c = TaskAllocationUtils::getCost(taskNum2, t, i2);
                    if (c < best) {
                        best = c;
                        bestTask = t;
                    }
                }
                if (bestTask != -1) {
                    std::cout << "[调试] 空闲AMR最小首段成本: amr=" << availableAmrs[i2].getDeviceId()
                              << ", task=" << taskList[bestTask].getMessageId()
                              << ", cost=" << best << " ms" << std::endl;
                } else {
                    std::cout << "[调试] 空闲AMR与所有任务之间均为INF/不可达: amr="
                              << availableAmrs[i2].getDeviceId() << std::endl;
                }
            }
        }
    }

    pathUpdater.requestExit();
    for (int i2 = 0; i2 < 200; ++i2) {
        if (pathUpdater.getActiveWorkers() == 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // 不兼容旧payload，不再打印旧字段

}

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

static std::string format_local_time() {
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    std::time_t t = static_cast<std::time_t>(ms.count() / 1000);
    int msPart = static_cast<int>(ms.count() % 1000);
    if (msPart < 0) {
        msPart += 1000;
        t -= 1;
    }
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec, msPart);
    return std::string(buf);
}

static std::string format_local_time_hhmmss_ms() {
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    std::time_t t = static_cast<std::time_t>(ms.count() / 1000);
    int msPart = static_cast<int>(ms.count() % 1000);
    if (msPart < 0) {
        msPart += 1000;
        t -= 1;
    }
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d%02d%02d%03d",
                  tm.tm_hour, tm.tm_min, tm.tm_sec, msPart);
    return std::string(buf);
}

static std::string log_time_prefix() {
    return "[" + format_local_time() + "] ";
}

static std::string format_nodes_head(const std::vector<int>& nodes, size_t maxCount = 5) {
    if (nodes.empty()) return "<empty>";
    std::ostringstream oss;
    const size_t limit = std::min(nodes.size(), maxCount);
    for (size_t i = 0; i < limit; ++i) {
        if (i > 0) oss << "->";
        oss << nodes[i];
    }
    if (nodes.size() > limit) oss << "...";
    return oss.str();
}

static std::string format_nodes_full(const std::vector<int>& nodes) {
    if (nodes.empty()) return "<empty>";
    std::ostringstream oss;
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (i > 0) oss << "->";
        oss << nodes[i];
    }
    return oss.str();
}

static const char* hold_reason_label(NodeReservationTable::HoldReason reason) {
    switch (reason) {
        case NodeReservationTable::HoldReason::RESERVED_PATH: return "reserved_path";
        case NodeReservationTable::HoldReason::WAITING_POINT: return "waiting_point";
        case NodeReservationTable::HoldReason::TEMP_GOAL: return "temp_goal";
        default: return "other";
    }
}

enum class LogLevel {
    ERROR = 0,
    WARN = 1,
    INFO = 2,
    DEBUG = 3,
    TRACE = 4,
};

static std::string lowercase(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

static LogLevel parse_log_level(const std::string& raw, LogLevel defv) {
    if (raw.empty()) return defv;
    std::string s = lowercase(trim_ascii(raw));
    if (s.empty()) return defv;
    const bool startupSummary = env_enabled("RECEIVER_LOG_STARTUP_SUMMARY");
    try {
        size_t idx = 0;
        int v = std::stoi(s, &idx);
        if (idx == s.size()) {
            v = std::max(0, std::min(4, v));
            return static_cast<LogLevel>(v);
        }
    } catch (...) {
    }
    if (s == "error") return LogLevel::ERROR;
    if (s == "warn" || s == "warning") return LogLevel::WARN;
    if (s == "info") return LogLevel::INFO;
    if (s == "debug") return LogLevel::DEBUG;
    if (s == "trace") return LogLevel::TRACE;
    return defv;
}

struct ReceiverLogConfig {
    LogLevel level = LogLevel::INFO;
    int statusLogIntervalSec = 0;
};

static ReceiverLogConfig load_receiver_log_config() {
    ReceiverLogConfig cfg;
    if (const char* v = std::getenv("LOG_LEVEL")) {
        cfg.level = parse_log_level(v, cfg.level);
    }
    if (const char* v = std::getenv("LOG_STATUS_INTERVAL_SEC")) {
        try { cfg.statusLogIntervalSec = std::max(0, std::stoi(v)); } catch (...) {}
    }
    return cfg;
}

static const ReceiverLogConfig& receiver_log_config() {
    static ReceiverLogConfig cfg = load_receiver_log_config();
    return cfg;
}

static bool log_enabled(LogLevel level) {
    return static_cast<int>(level) <= static_cast<int>(receiver_log_config().level);
}

class NullBuffer : public std::streambuf {
public:
    int overflow(int c) override { return c; }
};

static std::ostream& log_stream(LogLevel level) {
    static NullBuffer nullBuffer;
    static std::ostream nullStream(&nullBuffer);
    if (!log_enabled(level)) return nullStream;
    return (level <= LogLevel::WARN) ? std::cerr : std::cout;
}

static bool is_noisy_routing_key(const std::string& rk) {
    // 高频消息默认视为 noisy，仅在 TRACE 下输出“收到消息”日志。
    static const std::unordered_set<std::string> kNoisy = {
        "SendRobotStatusInfos",
        "SendRobotConfigInfos",
        "SimPose"
    };
    return kNoisy.count(rk) > 0;
}

static bool should_log_rx_header(const std::string& routingKey) {
    return log_enabled(is_noisy_routing_key(routingKey) ? LogLevel::TRACE : LogLevel::INFO);
}

static void maybe_log_status_update(size_t count) {
    if (count == 0) return;
    const auto& cfg = receiver_log_config();

    if (cfg.statusLogIntervalSec <= 0) {
        if (!log_enabled(LogLevel::TRACE)) return;
        std::cout << log_time_prefix() << "[Status] updated " << count << " robot status entries" << std::endl;
        return;
    }

    if (!log_enabled(LogLevel::DEBUG)) return;
    static std::chrono::steady_clock::time_point last = std::chrono::steady_clock::now();
    static size_t acc = 0;
    acc += count;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last).count();
    if (elapsed < cfg.statusLogIntervalSec) return;

    std::cout << log_time_prefix() << "[Status] updated +" << acc
              << " entries in last " << elapsed << "s" << std::endl;
    acc = 0;
    last = now;
}

struct StatusLagTracker {
    std::chrono::steady_clock::time_point lastLog{};
    long long totalMs = 0;
    long long count = 0;
    long long maxMs = 0;
    std::string maxDeviceId;
    long long missingTs = 0;
};

static void record_status_lag(const RobotStatusEntry& entry) {
    const int logIntervalSec = getenv_int("STATUS_LAG_LOG_SEC", 0);
    const int warnMs = getenv_int("STATUS_LAG_WARN_MS", 0);
    if (logIntervalSec <= 0 && warnMs <= 0) return;

    static StatusLagTracker tracker;
    if (tracker.lastLog == std::chrono::steady_clock::time_point{}) {
        tracker.lastLog = std::chrono::steady_clock::now();
    }

    bool hasTimestamp = entry.updateTime > 0;
    long long lagMs = 0;
    if (!hasTimestamp) {
        tracker.missingTs += 1;
    } else {
        const auto nowSec = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        lagMs = (nowSec - static_cast<long long>(entry.updateTime)) * 1000LL;
        if (lagMs < 0) lagMs = 0;
        tracker.totalMs += lagMs;
        tracker.count += 1;
        if (lagMs > tracker.maxMs) {
            tracker.maxMs = lagMs;
            tracker.maxDeviceId = entry.deviceId;
        }
        if (warnMs > 0 && lagMs >= warnMs) {
            std::cout << log_time_prefix()
                      << "[StatusLag] deviceId=" << entry.deviceId
                      << " lagMs=" << lagMs
                      << " updateTime=" << entry.updateTime
                      << std::endl;
        }
    }

    if (logIntervalSec <= 0) return;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.lastLog).count();
    if (elapsed < logIntervalSec) return;

    long long avgMs = (tracker.count > 0) ? (tracker.totalMs / tracker.count) : 0;
    std::cout << log_time_prefix()
              << "[StatusLag] intervalSec=" << elapsed
              << " count=" << tracker.count
              << " avgMs=" << avgMs
              << " maxMs=" << tracker.maxMs
              << " maxDevice=" << (tracker.maxDeviceId.empty() ? "<none>" : tracker.maxDeviceId)
              << " missingTs=" << tracker.missingTs
              << std::endl;
    tracker = StatusLagTracker{};
    tracker.lastLog = now;
}

static void log_line(LogLevel level, const std::string& message) {
    if (!log_enabled(level)) return;
    log_stream(level) << log_time_prefix() << message << std::endl;
}

static std::string iso8601_from_unix_millis(long long millis) {
    std::time_t seconds = static_cast<std::time_t>(millis / 1000);
    int msPart = static_cast<int>(millis % 1000);
    if (msPart < 0) {
        msPart += 1000;
        seconds -= 1;
    }
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &seconds);
#else
    gmtime_r(&seconds, &tm);
#endif
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec, msPart);
    return std::string(buf);
}

static std::string node_id_to_string(int nodeId) {
    return nodeId >= 0 ? std::to_string(nodeId) : std::string("");
}

static json make_node_point(const MapInfo& mapInfo, int nodeId, int yawDeg, int pointType) {
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
    pt["pointType"] = pointType;
    return pt;
}

static double compute_yaw_deg(double dx, double dy) {
    constexpr double kPi = 3.14159265358979323846;
    double yaw = std::atan2(dy, dx) * 180.0 / kPi;
    if (yaw < 0) yaw += 360.0;
    return yaw;
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

static std::optional<int> pick_any_neighbor_node_id(const MapInfo& mapInfo, int nodeId) {
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

static void dedup_consecutive_nodes(std::vector<int>& nodes) {
    if (nodes.size() < 2) return;
    std::vector<int> dedup;
    dedup.reserve(nodes.size());
    for (int n : nodes) {
        if (dedup.empty() || dedup.back() != n) dedup.push_back(n);
    }
    nodes.swap(dedup);
}

static bool normalize_route_min_nodes(const MapInfo& mapInfo,
                                      std::vector<int>& route,
                                      int fallbackNodeId,
                                      int minNodes,
                                      bool allowNeighborExtend) {
    if (route.empty() && fallbackNodeId >= 0) {
        route.push_back(fallbackNodeId);
    }
    for (int nodeId : route) {
        if (nodeId < 0) return false;
    }
    dedup_consecutive_nodes(route);
    if (route.empty()) return false;
    while (static_cast<int>(route.size()) < minNodes) {
        if (!allowNeighborExtend) return false;
        int last = route.back();
        auto nb = pick_any_neighbor_node_id(mapInfo, last);
        if (!nb.has_value() || *nb == last) return false;
        route.push_back(*nb);
        dedup_consecutive_nodes(route);
        if (route.empty()) return false;
    }
    return true;
}

static int parse_step_type_code(const std::string& stepType) {
    if (stepType.empty()) return -1;
    try {
        size_t consumed = 0;
        int code = std::stoi(stepType, &consumed);
        if (consumed > 0) return code;
    } catch (...) {}
    return TaskFieldUtils::PointTypeFromString(stepType);
}

static json build_segment_endpoint_point(const MapInfo& mapInfo,
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
    int yaw = resolve_point_yaw(mapInfo, route, idx);
    int pointType = startPoint ? -1 : parse_step_type_code(seg.stepType);
    return make_node_point(mapInfo, route[idx], yaw, pointType);
}

static std::string json_value_to_string(const json& v) {
    if (v.is_null()) return "";
    if (v.is_string()) return v.get<std::string>();
    if (v.is_number_integer()) return std::to_string(v.get<int>());
    if (v.is_number_unsigned()) return std::to_string(v.get<unsigned int>());
    if (v.is_number_float()) return std::to_string(v.get<double>());
    if (v.is_boolean()) return v.get<bool>() ? "true" : "false";
    return "";
}

static std::string read_string(const json& obj, const char* key, const std::string& def) {
    auto it = obj.find(key);
    if (it == obj.end()) return def;
    auto s = json_value_to_string(*it);
    return s.empty() ? def : s;
}

static int read_int(const json& obj, const char* key, int def) {
    auto it = obj.find(key);
    if (it == obj.end() || it->is_null()) return def;
    if (it->is_number_integer()) return it->get<int>();
    if (it->is_number_unsigned()) return static_cast<int>(it->get<unsigned int>());
    if (it->is_string()) {
        try { return std::stoi(it->get<std::string>()); } catch (...) {}
    }
    return def;
}

static double read_double(const json& obj, const char* key, double def) {
    auto it = obj.find(key);
    if (it == obj.end() || it->is_null()) return def;
    if (it->is_number()) return it->get<double>();
    if (it->is_string()) {
        try { return std::stod(it->get<std::string>()); } catch (...) {}
    }
    return def;
}

static bool read_bool(const json& obj, const char* key, bool def) {
    auto it = obj.find(key);
    if (it == obj.end() || it->is_null()) return def;
    if (it->is_boolean()) return it->get<bool>();
    if (it->is_number_integer()) return it->get<int>() != 0;
    if (it->is_string()) {
        std::string s = it->get<std::string>();
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
        if (s == "true" || s == "1" || s == "yes" || s == "on") return true;
        if (s == "false" || s == "0" || s == "no" || s == "off") return false;
    }
    return def;
}

static int parse_node_id_str(const std::string& s) {
    if (s.empty()) return -1;
    try { return std::stoi(s); } catch (...) { return -1; }
}

static double distance_mm_to_node_xy(const MapInfo& mapInfo,
                                     int nodeId,
                                     double x,
                                     double y) {
    if (nodeId < 0) {
        return std::numeric_limits<double>::infinity();
    }
    try {
        const Node& node = mapInfo.getNodeById(nodeId);
        const double dx = node.x - x;
        const double dy = node.y - y;
        return std::sqrt(dx * dx + dy * dy);
    } catch (...) {
        return std::numeric_limits<double>::infinity();
    }
}

static int resolve_status_node_id(const RobotStatusEntry& st, const MapInfo& mapInfo) {
    static std::mutex stickyMutex;
    static std::unordered_map<std::string, int> stickyNodeByDevice;

    auto read_sticky_node = [&](const std::string& deviceId) -> int {
        if (deviceId.empty()) return -1;
        std::lock_guard<std::mutex> lk(stickyMutex);
        auto it = stickyNodeByDevice.find(deviceId);
        if (it == stickyNodeByDevice.end()) return -1;
        return it->second;
    };

    auto write_sticky_node = [&](const std::string& deviceId, int nodeId) {
        if (deviceId.empty() || nodeId < 0) return;
        std::lock_guard<std::mutex> lk(stickyMutex);
        stickyNodeByDevice[deviceId] = nodeId;
    };

    auto clear_sticky_node = [&](const std::string& deviceId) {
        if (deviceId.empty()) return;
        std::lock_guard<std::mutex> lk(stickyMutex);
        stickyNodeByDevice.erase(deviceId);
    };

    int curNode = parse_node_id_str(st.nodeId);
    if (curNode >= 0) {
        write_sticky_node(st.deviceId, curNode);
        return curNode;
    }

    const double switchDistMm = std::max(0.0, getenv_double("STATUS_NODE_SWITCH_DIST_MM", 100.0));

    int x = static_cast<int>(std::lround(st.x));
    int y = static_cast<int>(std::lround(st.y));

    auto choose_candidate_with_threshold = [&](int candidateNodeId) -> int {
        if (candidateNodeId < 0) {
            return read_sticky_node(st.deviceId);
        }
        const double distMm = distance_mm_to_node_xy(mapInfo, candidateNodeId, st.x, st.y);
        if (distMm <= switchDistMm) {
            write_sticky_node(st.deviceId, candidateNodeId);
            return candidateNodeId;
        }
        int stickyNode = read_sticky_node(st.deviceId);
        if (stickyNode >= 0) {
            return stickyNode;
        }
        write_sticky_node(st.deviceId, candidateNodeId);
        return candidateNodeId;
    };

    int nextNodeId = -1;
    if (st.nextDestination.is_object()) {
        const auto& np = st.nextDestination;
        if (np.contains("nodeId")) {
            const auto& v = np["nodeId"];
            if (v.is_number_integer()) nextNodeId = v.get<int>();
            else if (v.is_string()) nextNodeId = parse_node_id_str(v.get<std::string>());
        }
        if (nextNodeId < 0) {
            int nx = np.value("x", x);
            int ny = np.value("y", y);
            try {
                int nearest = mapInfo.findNearestNodeId(nx, ny);
                return choose_candidate_with_threshold(nearest);
            } catch (...) {}
        }
    }
    if (nextNodeId >= 0) {
        try {
            int nearest = mapInfo.findNearestNodeIdFromNeighbors(x, y, nextNodeId);
            return choose_candidate_with_threshold(nearest);
        } catch (...) {}
    }
    try {
        int nearest = mapInfo.findNearestNodeId(x, y);
        return choose_candidate_with_threshold(nearest);
    } catch (...) {}

    int stickyNode = read_sticky_node(st.deviceId);
    if (stickyNode >= 0) return stickyNode;
    clear_sticky_node(st.deviceId);
    return -1;
}

static int resolve_status_next_node_id(const RobotStatusEntry& st, const MapInfo& mapInfo) {
    if (!st.nextDestination.is_object()) return -1;
    const auto& np = st.nextDestination;
    if (np.empty()) return -1;
    int nextNodeId = -1;
    if (np.contains("nodeId")) {
        const auto& v = np["nodeId"];
        if (v.is_number_integer()) nextNodeId = v.get<int>();
        else if (v.is_string()) nextNodeId = parse_node_id_str(v.get<std::string>());
    }
    if (nextNodeId >= 0) return nextNodeId;
    if (!np.contains("x") && !np.contains("y")) return -1;
    int nx = np.value("x", static_cast<int>(std::lround(st.x)));
    int ny = np.value("y", static_cast<int>(std::lround(st.y)));
    try { return mapInfo.findNearestNodeId(nx, ny); } catch (...) {}
    return -1;
}

static int resolve_committed_next_node_id(const RobotStatusEntry& st, const MapInfo& mapInfo) {
    if (!env_enabled("USE_STATUS_NEXT_NODE")) return -1;
    return resolve_status_next_node_id(st, mapInfo);
}

static std::vector<int> parse_env_node_list(const char* raw) {
    std::vector<int> out;
    if (!raw || !*raw) return out;
    std::string buf(raw);
    for (char& ch : buf) {
        unsigned char c = static_cast<unsigned char>(ch);
        if (!std::isdigit(c) && ch != '-' && ch != '+') ch = ' ';
    }
    std::istringstream iss(buf);
    int val = 0;
    while (iss >> val) {
        if (val >= 0) out.push_back(val);
    }
    return out;
}

static double distance_mm_between_nodes(const MapInfo& mapInfo, int fromNodeId, int toNodeId) {
    try {
        const Node& from = mapInfo.getNodeById(fromNodeId);
        const Node& to = mapInfo.getNodeById(toNodeId);
        return std::hypot(to.x - from.x, to.y - from.y);
    } catch (...) {
        return 0.0;
    }
}

static double compute_route_distance_mm(const MapInfo& mapInfo, const std::vector<int>& route) {
    if (route.size() < 2) return 0.0;
    double dist = 0.0;
    for (size_t i = 0; i + 1 < route.size(); ++i) {
        dist += distance_mm_between_nodes(mapInfo, route[i], route[i + 1]);
    }
    return dist;
}

static std::vector<int> build_route_from_plan(const PathPlanningHelper::AmrPlanInfo& plan);
static int find_route_index_at_or_after(const std::vector<int>& route,
                                        int nodeId,
                                        size_t startIndex);

static double compute_route_distance_mm_range(const MapInfo& mapInfo,
                                              const std::vector<int>& route,
                                              size_t startIdx,
                                              size_t endIdx) {
    if (route.size() < 2 || startIdx >= route.size()) return 0.0;
    if (endIdx >= route.size()) endIdx = route.size() - 1;
    if (endIdx <= startIdx) return 0.0;
    double dist = 0.0;
    for (size_t i = startIdx; i < endIdx; ++i) {
        dist += distance_mm_between_nodes(mapInfo, route[i], route[i + 1]);
    }
    return dist;
}

static double estimate_goal_distance_mm(const MapInfo& mapInfo,
                                        int startNodeId,
                                        int goalNodeId,
                                        AStarPathFinder* aStarPtr) {
    if (startNodeId < 0 || goalNodeId < 0) return -1.0;
    if (startNodeId == goalNodeId) return 0.0;
    if (aStarPtr != nullptr) {
        auto res = aStarPtr->findPath(
            startNodeId,
            goalNodeId,
            PathPlanningConstants::resolveTurnPenaltyMm(),
            nullptr,
            nullptr,
            nullptr,
            0.0);
        if (res.found && !res.path.empty()) {
            return compute_route_distance_mm(mapInfo, res.path);
        }
    }
    return distance_mm_between_nodes(mapInfo, startNodeId, goalNodeId);
}

static int compute_replan_interval_ms(double distMm, int baseIntervalMs) {
    const double spanMm = 50000.0;
    // Prefer explicit min/max range when provided (default: 500~1500ms).
    int minMs = getenv_int("REPLAN_INTERVAL_MIN_MS", -1);
    int maxMs = getenv_int("REPLAN_INTERVAL_MAX_MS", -1);
    if (minMs > 0 && maxMs > 0) {
        int farMs = std::min(minMs, maxMs);
        int nearMs = std::max(minMs, maxMs);
        if (distMm < 0.0) return farMs;
        if (distMm >= spanMm) return farMs;
        double t = distMm / spanMm;
        t = std::clamp(t, 0.0, 1.0);
        double ms = static_cast<double>(nearMs) - static_cast<double>(nearMs - farMs) * t;
        return static_cast<int>(std::llround(ms));
    }
    // Fallback to legacy behavior: near = 5x far.
    int farMs = std::max(1, baseIntervalMs);
    int nearMs = farMs * 5;
    if (distMm < 0.0) return farMs;
    if (distMm >= spanMm) return farMs;
    double t = distMm / spanMm;
    t = std::clamp(t, 0.0, 1.0);
    double ms = static_cast<double>(nearMs) - static_cast<double>(nearMs - farMs) * t;
    return static_cast<int>(std::llround(ms));
}

static double compute_reserve_scale_from_interval(int intervalMs, int baseMs) {
    if (baseMs <= 0) return 1.0;
    double scale = static_cast<double>(intervalMs) / static_cast<double>(baseMs);
    if (!std::isfinite(scale) || scale < 1.0) scale = 1.0;
    return scale;
}

struct ObstaclePointMm {
    double x = 0.0;
    double y = 0.0;
};

static bool point_to_segment_perp_distance_sq_mm_if_within(double px, double py,
                                                           double x1, double y1,
                                                           double x2, double y2,
                                                           double& outDist2) {
    outDist2 = 0.0;
    const double vx = x2 - x1;
    const double vy = y2 - y1;
    const double denom = vx * vx + vy * vy;
    if (denom <= 1e-9) {
        return false;
    }

    const double wx = px - x1;
    const double wy = py - y1;
    const double t = (wx * vx + wy * vy) / denom;

    // 仅当投影点落在线段范围内才认为“阻挡该边”；
    // 在线段延长线上（t<0 或 t>1）不计入边阻挡判定。
    if (t < 0.0 || t > 1.0) {
        return false;
    }

    const double projx = x1 + t * vx;
    const double projy = y1 + t * vy;
    const double dx = px - projx;
    const double dy = py - projy;
    outDist2 = dx * dx + dy * dy;
    return true;
}

struct BlockedGraph {
    std::set<int> blockedNodes;
    std::set<std::pair<int,int>> blockedEdges;
};

static BlockedGraph compute_blocked_graph_from_obstacles(
    const MapInfo& mapInfo,
    const std::vector<ObstaclePointMm>& obstacles,
    double nodeBlockMm = 300.0,
    double edgeBlockMm = 500.0
) {
    BlockedGraph out;
    if (obstacles.empty()) return out;

    const double nodeTh2 = nodeBlockMm * nodeBlockMm;
    const double edgeTh2 = edgeBlockMm * edgeBlockMm;

    const auto& nodes = mapInfo.getNodes();

    for (const auto& node : nodes) {
        if (node.id < 0) continue;
        for (const auto& ob : obstacles) {
            const double dx = node.x - ob.x;
            const double dy = node.y - ob.y;
            const double d2 = dx * dx + dy * dy;
            if (d2 <= nodeTh2) {
                out.blockedNodes.insert(node.id);
                break;
            }
        }
    }

    const auto& id2idx = mapInfo.getId2Index();
    const auto& edges = mapInfo.getEdges();
    for (const auto& edge : edges) {
        const int u = edge.startNodeId;
        const int v = edge.endNodeId;
        if (u < 0 || v < 0) continue;
        auto itU = id2idx.find(u);
        auto itV = id2idx.find(v);
        if (itU == id2idx.end() || itV == id2idx.end()) continue;
        const Node& nu = nodes[itU->second];
        const Node& nv = nodes[itV->second];
        for (const auto& ob : obstacles) {
            double d2 = 0.0;
            if (!point_to_segment_perp_distance_sq_mm_if_within(ob.x, ob.y, nu.x, nu.y, nv.x, nv.y, d2)) {
                continue;
            }
            if (d2 <= edgeTh2) {
                out.blockedEdges.insert({u, v});
                out.blockedEdges.insert({v, u});
                break;
            }
        }
    }

    return out;
}

static int compute_reserve_budget_nodes(int taskPriority, int batteryLevel, int deviceType) {
    const int minNodes = getenv_int("RESERVE_MIN_NODES", 2);
    const int maxNodes = getenv_int("RESERVE_MAX_NODES", 20);
    const int baseNodes = getenv_int("RESERVE_BASE_NODES", 4);
    double score = baseNodes;
    score += std::max(0, taskPriority) * 0.3;
    score += std::clamp(batteryLevel, 0, 100) * 0.05;
    if (deviceType > 0) score += 2.0;
    int n = static_cast<int>(std::lround(score));
    n = std::max(minNodes, n);
    n = std::min(maxNodes, n);
    return n;
}

static bool is_node_targeted_by_other(RobotDataRepository& repo,
                                      const std::string& selfId,
                                      int nodeId) {
    if (nodeId < 0) return false;
    auto plans = repo.snapshotPlans();
    for (const auto& kv : plans) {
        if (kv.first == selfId) continue;
        const auto& segs = kv.second.plan.segments;
        for (const auto& seg : segs) {
            if (seg.toNodeId == nodeId) return true;
        }
    }
    return false;
}

struct CongestionInfo {
    std::unordered_map<int, double> nodeScore;
    std::unordered_set<int> hardBlocked;
    std::unordered_set<int> hotNodes;
};

struct RegionCongestionInfo {
    bool useGraph = false;
    int regionCount = 0;
    int grid = 0;
    int window = 0;
    int bigWindow = 0;
    int softLimit = 0;
    int hardLimit = 0;
    int bigHardLimit = 0;
    double minX = 0.0;
    double maxX = 0.0;
    double minY = 0.0;
    double maxY = 0.0;
    double cellW = 0.0;
    double cellH = 0.0;
    std::vector<int> regionByIndex;
    std::unordered_map<int, double> nodeScore;
    std::unordered_set<int> hardBlockedNodes;

    bool valid() const {
        if (useGraph) return regionCount > 0;
        return grid > 0 && cellW > 0.0 && cellH > 0.0;
    }

    int cellIndex(double x, double y) const {
        if (!valid() || useGraph) return -1;
        int cx = static_cast<int>(std::floor((x - minX) / cellW));
        int cy = static_cast<int>(std::floor((y - minY) / cellH));
        if (cx < 0) cx = 0;
        if (cy < 0) cy = 0;
        if (cx >= grid) cx = grid - 1;
        if (cy >= grid) cy = grid - 1;
        return cy * grid + cx;
    }

    int cellIndexForNode(const MapInfo& mapInfo, int nodeId) const {
        if (nodeId < 0) return -1;
        if (useGraph && !regionByIndex.empty()) {
            const auto& id2idx = mapInfo.getId2Index();
            auto it = id2idx.find(nodeId);
            if (it == id2idx.end()) return -1;
            int idx = it->second;
            if (idx < 0 || idx >= static_cast<int>(regionByIndex.size())) return -1;
            return regionByIndex[static_cast<size_t>(idx)];
        }
        try {
            const Node& node = mapInfo.getNodeById(nodeId);
            return cellIndex(node.x, node.y);
        } catch (...) {
            return -1;
        }
    }
};

static bool compute_map_bounds(const MapInfo& mapInfo,
                               double& minX,
                               double& maxX,
                               double& minY,
                               double& maxY) {
    const auto& nodes = mapInfo.getNodes();
    bool has = false;
    for (const auto& node : nodes) {
        if (node.id < 0) continue;
        if (!has) {
            minX = maxX = node.x;
            minY = maxY = node.y;
            has = true;
            continue;
        }
        minX = std::min(minX, node.x);
        maxX = std::max(maxX, node.x);
        minY = std::min(minY, node.y);
        maxY = std::max(maxY, node.y);
    }
    return has;
}

static int auto_region_grid(int agvCount) {
    int minGrid = 3;
    int maxGrid = 6;
    const int envGridMin = getenv_int("CONGESTION_REGION_GRID_MIN", 0);
    const int envGridMax = getenv_int("CONGESTION_REGION_GRID_MAX", 0);
    if (envGridMin > 0) minGrid = envGridMin;
    if (envGridMax > 0) maxGrid = envGridMax;
    if (maxGrid < minGrid) maxGrid = minGrid;
    int grid = static_cast<int>(std::lround(std::sqrt(std::max(1, agvCount))));
    grid = std::clamp(grid, minGrid, maxGrid);
    return grid;
}

static bool is_passable_region_node(const Node& node) {
    return CongestionRegionUtils::isPassableRegionNode(node);
}

static void gather_neighbors_indices(const MapInfo& mapInfo,
                                     int idx,
                                     std::vector<int>& out) {
    CongestionRegionUtils::gatherNeighborIndices(mapInfo, idx, out);
}

static int build_graph_regions(const MapInfo& mapInfo,
                               int targetRegions,
                               std::vector<int>& regionByIndex,
                               std::vector<int>& seedIndices,
                               std::vector<char>* bridgeRegionMask,
                               bool enableBridgeRegions) {
    return CongestionRegionUtils::buildGraphRegions(
        mapInfo,
        targetRegions,
        regionByIndex,
        seedIndices,
        bridgeRegionMask,
        enableBridgeRegions);
}

static RegionCongestionInfo build_region_congestion_info(
    const MapInfo& mapInfo,
    const std::vector<RobotStatusEntry>& statuses) {
    RegionCongestionInfo info;
    std::vector<int> nodeIds;
    nodeIds.reserve(statuses.size());
    for (const auto& st : statuses) {
        if (st.deviceId.empty()) continue;
        if (!st.connection) continue;
        int nodeId = resolve_status_node_id(st, mapInfo);
        if (nodeId < 0) continue;
        nodeIds.push_back(nodeId);
    }
    const int agvCount = static_cast<int>(nodeIds.size());
    if (agvCount <= 0) return info;

    int grid = getenv_int("CONGESTION_REGION_GRID", 0);
    if (grid <= 0) grid = auto_region_grid(agvCount);
    if (grid <= 0) return info;

    int window = getenv_int("CONGESTION_REGION_WINDOW", 3);
    if (window < 1) window = 1;
    if (window % 2 == 0) window += 1;
    int bigWindow = getenv_int("CONGESTION_REGION_BIG_WINDOW", 3);
    if (bigWindow < 1) bigWindow = 1;
    if (bigWindow % 2 == 0) bigWindow += 1;

    std::string mode = to_lower_ascii(getenv_str("CONGESTION_REGION_MODE", "graph"));
    bool useGraph = (mode == "graph" || mode == "cluster" || mode == "partition");
    info.useGraph = useGraph;
    info.grid = grid;
    info.window = window;
    info.bigWindow = bigWindow;

    if (useGraph) {
        int regionCount = getenv_int("CONGESTION_REGION_COUNT", 0);
        if (regionCount <= 0) regionCount = grid * grid;
        const bool enableBridgeRegions = getenv_int("CONGESTION_REGION_BRIDGE_ENABLE", 1) != 0;
        std::vector<int> regionByIndex;
        std::vector<int> seedIndices;
        std::vector<char> bridgeRegionMask;
        regionCount = build_graph_regions(mapInfo,
                                          regionCount,
                                          regionByIndex,
                                          seedIndices,
                                          &bridgeRegionMask,
                                          enableBridgeRegions);
        if (regionCount <= 0) return info;

        info.regionCount = regionCount;
        info.regionByIndex = regionByIndex;

        const auto& nodes = mapInfo.getNodes();
        const auto& id2idx = mapInfo.getId2Index();
        std::vector<int> regionCounts(regionCount, 0);
        for (int nodeId : nodeIds) {
            auto it = id2idx.find(nodeId);
            if (it == id2idx.end()) continue;
            int idx = it->second;
            if (idx < 0 || idx >= static_cast<int>(regionByIndex.size())) continue;
            int reg = regionByIndex[static_cast<size_t>(idx)];
            if (reg >= 0 && reg < regionCount) regionCounts[reg] += 1;
        }

        std::vector<int> regionNodeCounts(regionCount, 0);
        int totalNormalNodes = 0;
        int normalAgvCount = 0;
        for (size_t i = 0; i < nodes.size(); ++i) {
            const Node& node = nodes[i];
            if (!is_passable_region_node(node)) continue;
            int reg = regionByIndex[i];
            if (reg < 0 || reg >= regionCount) continue;
            regionNodeCounts[reg] += 1;
        }
        int effectiveNormalRegions = 0;
        for (int r = 0; r < regionCount; ++r) {
            if (r < static_cast<int>(bridgeRegionMask.size()) && bridgeRegionMask[static_cast<size_t>(r)]) {
                continue;
            }
            if (regionNodeCounts[r] > 0) {
                effectiveNormalRegions += 1;
                totalNormalNodes += regionNodeCounts[r];
            }
            normalAgvCount += regionCounts[r];
        }
        if (effectiveNormalRegions <= 0) effectiveNormalRegions = 1;

        double avg = static_cast<double>(normalAgvCount) /
                     static_cast<double>(effectiveNormalRegions);
        int softLimit = getenv_int("CONGESTION_REGION_SOFT", 0);
        int hardLimit = getenv_int("CONGESTION_REGION_HARD", 0);
        if (softLimit <= 0) {
            softLimit = std::max(2, static_cast<int>(std::ceil(avg)) + 1);
        }
        if (hardLimit <= 0) {
            int bump = std::max(1, static_cast<int>(std::ceil(avg / 2.0)));
            hardLimit = softLimit + bump;
        }
        if (hardLimit <= softLimit) hardLimit = softLimit + 1;

        info.softLimit = softLimit;
        info.hardLimit = hardLimit;

        double avgNodes = static_cast<double>(totalNormalNodes) /
                          static_cast<double>(std::max(1, effectiveNormalRegions));
        std::vector<int> regionSoft(regionCount, 0);
        std::vector<int> regionHard(regionCount, 0);
        for (int r = 0; r < regionCount; ++r) {
            if (r < static_cast<int>(bridgeRegionMask.size()) && bridgeRegionMask[static_cast<size_t>(r)]) {
                regionSoft[r] = 0;
                regionHard[r] = 1;
                continue;
            }
            int nodeCount = regionNodeCounts[r];
            if (nodeCount <= 0) {
                regionSoft[r] = 0;
                regionHard[r] = 0;
                continue;
            }
            double factor = (avgNodes > 0.0)
                ? (static_cast<double>(nodeCount) / avgNodes)
                : 1.0;
            int s = std::max(1, static_cast<int>(std::lround(softLimit * factor)));
            int h = std::max(1, static_cast<int>(std::lround(hardLimit * factor)));
            if (h <= s) h = s + 1;
            regionSoft[r] = s;
            regionHard[r] = h;
        }

        std::vector<std::vector<int>> regionAdj(regionCount);
        std::vector<std::vector<char>> adjMat(regionCount, std::vector<char>(regionCount, 0));
        std::vector<int> neighbors;
        for (int idx = 0; idx < static_cast<int>(nodes.size()); ++idx) {
            if (!is_passable_region_node(nodes[static_cast<size_t>(idx)])) continue;
            int reg = regionByIndex[static_cast<size_t>(idx)];
            if (reg < 0 || reg >= regionCount) continue;
            gather_neighbors_indices(mapInfo, idx, neighbors);
            for (int nb : neighbors) {
                if (nb < 0 || nb >= static_cast<int>(nodes.size())) continue;
                if (!is_passable_region_node(nodes[static_cast<size_t>(nb)])) continue;
                int reg2 = regionByIndex[static_cast<size_t>(nb)];
                if (reg2 < 0 || reg2 >= regionCount || reg2 == reg) continue;
                if (!adjMat[reg][reg2]) {
                    adjMat[reg][reg2] = 1;
                    adjMat[reg2][reg] = 1;
                    regionAdj[reg].push_back(reg2);
                    regionAdj[reg2].push_back(reg);
                }
            }
        }

        int radius = window / 2;
        int bigRadius = bigWindow / 2;
        double bigRatio = getenv_double("CONGESTION_REGION_BIG_RATIO", 0.8);
        if (!(bigRatio > 0.0)) bigRatio = 0.8;

        std::vector<int> windowCounts(regionCount, 0);
        std::vector<int> windowSoft(regionCount, 0);
        std::vector<int> windowHard(regionCount, 0);
        std::vector<int> bigCounts;
        std::vector<int> bigHard;
        if (bigWindow > 1) {
            bigCounts.assign(regionCount, 0);
            bigHard.assign(regionCount, 0);
        }

        auto accumulate_region = [&](int start,
                                     int radius,
                                     int& outCount,
                                     int& outSoft,
                                     int& outHard) {
            outCount = 0;
            outSoft = 0;
            outHard = 0;
            std::vector<int> dist(regionCount, -1);
            std::deque<int> q;
            dist[start] = 0;
            q.push_back(start);
            while (!q.empty()) {
                int cur = q.front();
                q.pop_front();
                outCount += regionCounts[cur];
                outSoft += regionSoft[cur];
                outHard += regionHard[cur];
                if (dist[cur] >= radius) continue;
                for (int nb : regionAdj[cur]) {
                    if (nb < 0 || nb >= regionCount) continue;
                    if (dist[nb] >= 0) continue;
                    dist[nb] = dist[cur] + 1;
                    q.push_back(nb);
                }
            }
        };

        for (int r = 0; r < regionCount; ++r) {
            int sumCount = 0;
            int sumSoft = 0;
            int sumHard = 0;
            accumulate_region(r, radius, sumCount, sumSoft, sumHard);
            if (sumHard <= sumSoft) sumHard = sumSoft + 1;
            if (sumHard <= 0) sumHard = 1;
            windowCounts[r] = sumCount;
            windowSoft[r] = sumSoft;
            windowHard[r] = sumHard;

            if (bigWindow > 1) {
                int bCount = 0;
                int bSoft = 0;
                int bHardSum = 0;
                accumulate_region(r, bigRadius, bCount, bSoft, bHardSum);
                int hardCap = static_cast<int>(std::floor(static_cast<double>(bHardSum) * bigRatio));
                if (hardCap < 1) hardCap = 1;
                bigCounts[r] = bCount;
                bigHard[r] = hardCap;
            }
        }

        std::vector<double> regionScore(regionCount, 0.0);
        for (int r = 0; r < regionCount; ++r) {
            const bool isBridgeRegion =
                (r < static_cast<int>(bridgeRegionMask.size()) &&
                 bridgeRegionMask[static_cast<size_t>(r)]);
            int count = isBridgeRegion ? regionCounts[r] : windowCounts[r];
            int soft = isBridgeRegion ? regionSoft[r] : windowSoft[r];
            int hard = isBridgeRegion ? regionHard[r] : windowHard[r];
            double score = 0.0;
            if (count >= hard) {
                score = 1.0;
            } else if (count > soft && hard > soft) {
                score = static_cast<double>(count - soft) /
                        static_cast<double>(hard - soft);
            }
            regionScore[r] = std::clamp(score, 0.0, 1.0);
        }

        for (size_t i = 0; i < nodes.size(); ++i) {
            const Node& node = nodes[i];
            if (!is_passable_region_node(node)) continue;
            int reg = regionByIndex[i];
            if (reg < 0 || reg >= regionCount) continue;
            double score = regionScore[reg];
            if (score > 0.0) {
                info.nodeScore[node.id] = score;
            }
            const bool isBridgeRegion =
                (reg < static_cast<int>(bridgeRegionMask.size()) &&
                 bridgeRegionMask[static_cast<size_t>(reg)]);
            bool hardBlock = isBridgeRegion
                ? (regionCounts[reg] >= regionHard[reg])
                : (windowCounts[reg] >= windowHard[reg]);
            if (!isBridgeRegion && !bigCounts.empty() && bigCounts[reg] >= bigHard[reg]) hardBlock = true;
            if (hardBlock) info.hardBlockedNodes.insert(node.id);
        }
        return info;
    }

    double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0;
    if (!compute_map_bounds(mapInfo, minX, maxX, minY, maxY)) {
        return info;
    }
    double spanX = std::max(1e-6, maxX - minX);
    double spanY = std::max(1e-6, maxY - minY);
    double cellW = spanX / static_cast<double>(grid);
    double cellH = spanY / static_cast<double>(grid);

    std::vector<int> cellCounts(static_cast<size_t>(grid * grid), 0);
    for (int nodeId : nodeIds) {
        try {
            const Node& node = mapInfo.getNodeById(nodeId);
            int cell = static_cast<int>(std::floor((node.x - minX) / cellW));
            int row = static_cast<int>(std::floor((node.y - minY) / cellH));
            if (cell < 0) cell = 0;
            if (row < 0) row = 0;
            if (cell >= grid) cell = grid - 1;
            if (row >= grid) row = grid - 1;
            cellCounts[static_cast<size_t>(row * grid + cell)] += 1;
        } catch (...) {
            continue;
        }
    }

    std::vector<int> cellNodeCounts(static_cast<size_t>(grid * grid), 0);
    int totalNodes = 0;
    const auto& nodes = mapInfo.getNodes();
    for (const auto& node : nodes) {
        if (!is_passable_region_node(node)) continue;
        int cell = static_cast<int>(std::floor((node.x - minX) / cellW));
        int row = static_cast<int>(std::floor((node.y - minY) / cellH));
        if (cell < 0) cell = 0;
        if (row < 0) row = 0;
        if (cell >= grid) cell = grid - 1;
        if (row >= grid) row = grid - 1;
        cellNodeCounts[static_cast<size_t>(row * grid + cell)] += 1;
        totalNodes += 1;
    }
    int effectiveCells = 0;
    for (int c : cellNodeCounts) {
        if (c > 0) effectiveCells++;
    }
    if (effectiveCells <= 0) effectiveCells = grid * grid;

    double avg = static_cast<double>(agvCount) / static_cast<double>(effectiveCells);
    int softLimit = getenv_int("CONGESTION_REGION_SOFT", 0);
    int hardLimit = getenv_int("CONGESTION_REGION_HARD", 0);
    if (softLimit <= 0) {
        softLimit = std::max(2, static_cast<int>(std::ceil(avg)) + 1);
    }
    if (hardLimit <= 0) {
        int bump = std::max(1, static_cast<int>(std::ceil(avg / 2.0)));
        hardLimit = softLimit + bump;
    }
    if (hardLimit <= softLimit) hardLimit = softLimit + 1;

    double avgNodes = static_cast<double>(totalNodes) /
                      static_cast<double>(std::max(1, effectiveCells));
    std::vector<int> cellSoft(static_cast<size_t>(grid * grid), 0);
    std::vector<int> cellHard(static_cast<size_t>(grid * grid), 0);
    for (size_t i = 0; i < cellSoft.size(); ++i) {
        int nodeCount = cellNodeCounts[i];
        if (nodeCount <= 0) {
            cellSoft[i] = 0;
            cellHard[i] = 0;
            continue;
        }
        double factor = (avgNodes > 0.0)
            ? (static_cast<double>(nodeCount) / avgNodes)
            : 1.0;
        int s = std::max(1, static_cast<int>(std::lround(softLimit * factor)));
        int h = std::max(1, static_cast<int>(std::lround(hardLimit * factor)));
        if (h <= s) h = s + 1;
        cellSoft[i] = s;
        cellHard[i] = h;
    }

    std::vector<int> windowCounts(static_cast<size_t>(grid * grid), 0);
    std::vector<int> windowSoft(static_cast<size_t>(grid * grid), 0);
    std::vector<int> windowHard(static_cast<size_t>(grid * grid), 0);
    const int half = window / 2;
    for (int row = 0; row < grid; ++row) {
        for (int col = 0; col < grid; ++col) {
            int sumCount = 0;
            int sumSoft = 0;
            int sumHard = 0;
            for (int dy = -half; dy <= half; ++dy) {
                int ny = row + dy;
                if (ny < 0 || ny >= grid) continue;
                for (int dx = -half; dx <= half; ++dx) {
                    int nx = col + dx;
                    if (nx < 0 || nx >= grid) continue;
                    size_t idx = static_cast<size_t>(ny * grid + nx);
                    sumCount += cellCounts[idx];
                    sumSoft += cellSoft[idx];
                    sumHard += cellHard[idx];
                }
            }
            if (sumHard <= sumSoft) sumHard = sumSoft + 1;
            if (sumHard <= 0) sumHard = 1;
            size_t idx = static_cast<size_t>(row * grid + col);
            windowCounts[idx] = sumCount;
            windowSoft[idx] = sumSoft;
            windowHard[idx] = sumHard;
        }
    }

    std::vector<int> bigWindowCounts;
    std::vector<int> bigWindowHard;
    double bigRatio = getenv_double("CONGESTION_REGION_BIG_RATIO", 0.8);
    if (!(bigRatio > 0.0)) bigRatio = 0.8;
    int bigHardLimit = 0;
    if (bigWindow > 1) {
        bigHardLimit = getenv_int("CONGESTION_REGION_BIG_HARD", 0);
        if (bigHardLimit <= 0) {
            const int bigCells = bigWindow * bigWindow;
            const double base = static_cast<double>(hardLimit) * static_cast<double>(bigCells);
            bigHardLimit = static_cast<int>(std::floor(base * bigRatio));
        }
        if (bigHardLimit < 1) bigHardLimit = 1;
        bigWindowCounts.assign(static_cast<size_t>(grid * grid), 0);
        bigWindowHard.assign(static_cast<size_t>(grid * grid), 0);
        const int bigHalf = bigWindow / 2;
        for (int row = 0; row < grid; ++row) {
            for (int col = 0; col < grid; ++col) {
                int sumCount = 0;
                int sumHard = 0;
                for (int dy = -bigHalf; dy <= bigHalf; ++dy) {
                    int ny = row + dy;
                    if (ny < 0 || ny >= grid) continue;
                    for (int dx = -bigHalf; dx <= bigHalf; ++dx) {
                        int nx = col + dx;
                        if (nx < 0 || nx >= grid) continue;
                        size_t idx = static_cast<size_t>(ny * grid + nx);
                        sumCount += cellCounts[idx];
                        sumHard += cellHard[idx];
                    }
                }
                size_t idx = static_cast<size_t>(row * grid + col);
                bigWindowCounts[idx] = sumCount;
                int hardCap = static_cast<int>(std::floor(static_cast<double>(sumHard) * bigRatio));
                if (hardCap < 1) hardCap = 1;
                bigWindowHard[idx] = hardCap;
            }
        }
    }

    std::vector<double> cellScore(static_cast<size_t>(grid * grid), 0.0);
    for (size_t i = 0; i < cellScore.size(); ++i) {
        int count = windowCounts[i];
        int soft = windowSoft[i];
        int hard = windowHard[i];
        double score = 0.0;
        if (count >= hard) {
            score = 1.0;
        } else if (count > soft && hard > soft) {
            score = static_cast<double>(count - soft) /
                    static_cast<double>(hard - soft);
        }
        cellScore[i] = std::clamp(score, 0.0, 1.0);
    }

    info.softLimit = softLimit;
    info.hardLimit = hardLimit;
    info.bigHardLimit = bigHardLimit;
    info.minX = minX;
    info.maxX = maxX;
    info.minY = minY;
    info.maxY = maxY;
    info.cellW = cellW;
    info.cellH = cellH;

    for (const auto& node : nodes) {
        if (!is_passable_region_node(node)) continue;
        int cell = info.cellIndex(node.x, node.y);
        if (cell < 0) continue;
        double score = cellScore[static_cast<size_t>(cell)];
        if (score > 0.0) {
            info.nodeScore[node.id] = score;
        }
        bool hardBlock = (windowCounts[static_cast<size_t>(cell)] >= windowHard[static_cast<size_t>(cell)]);
        if (!bigWindowCounts.empty() &&
            bigWindowCounts[static_cast<size_t>(cell)] >= bigWindowHard[static_cast<size_t>(cell)]) {
            hardBlock = true;
        }
        if (hardBlock) {
            info.hardBlockedNodes.insert(node.id);
        }
    }
    return info;
}

static void merge_region_congestion(const RegionCongestionInfo& region,
                                    CongestionInfo& congestion) {
    if (!region.valid()) return;
    for (const auto& kv : region.nodeScore) {
        auto it = congestion.nodeScore.find(kv.first);
        if (it == congestion.nodeScore.end() || kv.second > it->second) {
            congestion.nodeScore[kv.first] = kv.second;
        }
    }
    // Region hard-block is extremely aggressive (can easily make paths "unreachable" in dense sims).
    // Keep it as a penalty by default; opt-in to hard obstacles via env.
    if (env_enabled_default_true("CONGESTION_REGION_HARD_BLOCK")) {
        congestion.hardBlocked.insert(region.hardBlockedNodes.begin(), region.hardBlockedNodes.end());
    }
}

static std::vector<int> filter_hard_blocked_nodes_for_start(
    const std::unordered_set<int>& hardBlocked,
    const RegionCongestionInfo& region,
    const MapInfo& mapInfo,
    const NodeReservationTable& reservations,
    int startNodeId) {
    std::vector<int> out;
    if (hardBlocked.empty()) return out;
    if (!region.valid()) {
        out.insert(out.end(), hardBlocked.begin(), hardBlocked.end());
        return out;
    }
    int startCell = region.cellIndexForNode(mapInfo, startNodeId);
    for (int nodeId : hardBlocked) {
        // Never "unblock" a node that is actively held by another AGV, even if it is in the same
        // congestion cell as the start. Otherwise the planner will repeatedly pick occupied
        // neighbors (WAITING_POINT) and the reservation step will immediately truncate to 1 point.
        if (reservations.findBlockingHold(nodeId).has_value()) {
            out.push_back(nodeId);
            continue;
        }
        int cell = region.cellIndexForNode(mapInfo, nodeId);
        if (startCell >= 0 && cell == startCell) continue;
        out.push_back(nodeId);
    }
    return out;
}
static void gather_neighbors_by_id(const MapInfo& mapInfo,
                                   int nodeId,
                                   std::vector<int>& out) {
    out.clear();
    const auto& id2idx = mapInfo.getId2Index();
    auto it = id2idx.find(nodeId);
    if (it == id2idx.end()) return;
    const int idx = it->second;
    const auto& aft = mapInfo.getAftNode();
    auto itA = aft.find(idx);
    if (itA == aft.end()) return;
    const auto& nodes = mapInfo.getNodes();
    for (int nidx : itA->second) {
        if (nidx < 0 || nidx >= static_cast<int>(nodes.size())) continue;
        out.push_back(nodes[nidx].id);
    }
}

static void expand_congestion_scores(const MapInfo& mapInfo,
                                     int startNodeId,
                                     double baseWeight,
                                     int maxHops,
                                     double decay,
                                     std::unordered_map<int, double>& scores) {
    if (maxHops <= 0 || !(decay > 0.0)) return;
    std::deque<std::pair<int, int>> q;
    std::unordered_set<int> visited;
    visited.insert(startNodeId);
    q.push_back({startNodeId, 0});
    std::vector<int> neighbors;
    while (!q.empty()) {
        auto [nodeId, depth] = q.front();
        q.pop_front();
        if (depth >= maxHops) continue;
        gather_neighbors_by_id(mapInfo, nodeId, neighbors);
        for (int nb : neighbors) {
            if (visited.insert(nb).second) {
                int nextDepth = depth + 1;
                double w = baseWeight * std::pow(decay, static_cast<double>(nextDepth));
                auto it = scores.find(nb);
                if (it == scores.end() || w > it->second) {
                    scores[nb] = w;
                }
                q.push_back({nb, nextDepth});
            }
        }
    }
}

static CongestionInfo build_congestion_info(const MapInfo& mapInfo,
                                            const NodeReservationTable& reservations,
                                            const std::vector<int>& blockedNodes,
                                            std::chrono::steady_clock::time_point now) {
    CongestionInfo info;
    if (blockedNodes.empty()) return info;

    const int hotSec = getenv_int("CONGESTION_HOT_SEC", 5);
    const int expandHops = getenv_int("CONGESTION_EXPAND_HOPS", 1);
    const double decay = getenv_double("CONGESTION_DECAY", 0.6);
    const double wReserved = getenv_double("CONGESTION_RESERVED_WEIGHT", 0.35);
    const double wWaiting = getenv_double("CONGESTION_WAIT_WEIGHT", 1.0);
    const double wTemp = getenv_double("CONGESTION_TEMP_WEIGHT", 1.0);
    const double wHot = getenv_double("CONGESTION_HOT_WEIGHT", 1.0);
    double blockThreshold = getenv_double("CONGESTION_BLOCK_THRESHOLD", 0.85);
    if (!(blockThreshold > 0.0)) blockThreshold = 0.85;

    for (int nodeId : blockedNodes) {
        double weight = wReserved;
        bool hot = false;
        if (auto entry = reservations.findBlockingHold(nodeId)) {
            if (entry->reason == NodeReservationTable::HoldReason::WAITING_POINT) {
                weight = wWaiting;
            } else if (entry->reason == NodeReservationTable::HoldReason::TEMP_GOAL) {
                weight = wTemp;
            }
            if (hotSec > 0) {
                auto age = std::chrono::duration_cast<std::chrono::seconds>(now - entry->updatedAt).count();
                if (age >= hotSec) {
                    weight = std::max(weight, wHot);
                    hot = true;
                }
            }
        }
        auto it = info.nodeScore.find(nodeId);
        if (it == info.nodeScore.end() || weight > it->second) {
            info.nodeScore[nodeId] = weight;
        }
        if (hot) info.hotNodes.insert(nodeId);
        expand_congestion_scores(mapInfo, nodeId, weight, expandHops, decay, info.nodeScore);
    }

    for (const auto& kv : info.nodeScore) {
        if (kv.second >= blockThreshold) {
            info.hardBlocked.insert(kv.first);
        }
    }
    return info;
}

static std::optional<int> select_temp_goal_node(const MapInfo& mapInfo,
                                                int currentNodeId,
                                                const std::unordered_set<int>& bannedNodes,
                                                const std::unordered_set<int>& congestionNodes,
                                                const std::unordered_map<int, double>* nodeScore,
                                                double lowScoreThreshold,
                                                const RegionCongestionInfo* region,
                                                int goalNodeId) {
    if (currentNodeId < 0) return std::nullopt;

    double cx = 0.0, cy = 0.0;
    double gx = 0.0, gy = 0.0;
    bool hasGoal = false;
    const double nearGoalMm = std::max(0.0, getenv_double("TEMP_GOAL_NEAR_MM", 8000.0));
    const double nearGoalMm2 = nearGoalMm * nearGoalMm;
    const auto& nodes = mapInfo.getNodes();
    int currentCell = -1;
    try {
        const Node& cur = mapInfo.getNodeById(currentNodeId);
        cx = cur.x;
        cy = cur.y;
        if (region && region->valid()) {
            currentCell = region->cellIndexForNode(mapInfo, currentNodeId);
        }
    } catch (...) {
        return std::nullopt;
    }
    if (goalNodeId >= 0 && nearGoalMm > 0.0) {
        try {
            const Node& goal = mapInfo.getNodeById(goalNodeId);
            gx = goal.x;
            gy = goal.y;
            hasGoal = true;
        } catch (...) {
            hasGoal = false;
        }
    }
    const bool useRegion = (region && region->valid() && currentCell >= 0);

    std::vector<std::pair<double, double>> congestionPos;
    if (!congestionNodes.empty()) {
        congestionPos.reserve(congestionNodes.size());
        for (int cid : congestionNodes) {
            try {
                const Node& cn = mapInfo.getNodeById(cid);
                congestionPos.push_back({cn.x, cn.y});
            } catch (...) {
                continue;
            }
        }
    }

    auto score_of = [&](int nodeId) -> double {
        if (!nodeScore) return 0.0;
        auto it = nodeScore->find(nodeId);
        if (it == nodeScore->end()) return 0.0;
        return std::clamp(it->second, 0.0, 1.0);
    };

    auto consider_node = [&](int nodeId,
                             double nx,
                             double ny,
                             bool requireLow,
                             bool requireDifferentRegion,
                             bool requireNearGoal,
                             double& bestScore,
                             double& bestCongest,
                             double& bestDist,
                             int& bestNode) {
        if (nodeId < 0 || nodeId == currentNodeId) return;
        if (bannedNodes.count(nodeId)) return;
        if (congestionNodes.count(nodeId)) return;
        if (requireDifferentRegion && useRegion) {
            int cell = region->cellIndexForNode(mapInfo, nodeId);
            if (cell == currentCell) return;
        }
        if (requireNearGoal) {
            if (!hasGoal) return;
            double gdx = nx - gx;
            double gdy = ny - gy;
            double g2 = gdx * gdx + gdy * gdy;
            if (g2 > nearGoalMm2) return;
        }
        double score = score_of(nodeId);
        if (requireLow && score > lowScoreThreshold) return;
        double ax = requireNearGoal ? gx : cx;
        double ay = requireNearGoal ? gy : cy;
        double dx = nx - ax;
        double dy = ny - ay;
        double d2 = dx * dx + dy * dy;
        double minC = 0.0;
        if (!congestionPos.empty()) {
            minC = std::numeric_limits<double>::infinity();
            for (const auto& pos : congestionPos) {
                double cdx = nx - pos.first;
                double cdy = ny - pos.second;
                double cd2 = cdx * cdx + cdy * cdy;
                if (cd2 < minC) minC = cd2;
            }
            if (!(minC < std::numeric_limits<double>::infinity())) return;
        }
        bool better = false;
        if (score + 1e-9 < bestScore) {
            better = true;
        } else if (std::abs(score - bestScore) <= 1e-9) {
            if (!congestionPos.empty()) {
                if (minC > bestCongest) better = true;
            } else if (d2 < bestDist) {
                better = true;
            }
        }
        if (better) {
            bestScore = score;
            bestCongest = minC;
            bestDist = d2;
            bestNode = nodeId;
        }
    };

    auto pick_from_list = [&](const std::vector<int>& candidates,
                              bool requireLow,
                              bool requireDifferentRegion,
                              bool requireNearGoal) -> int {
        double bestScore = std::numeric_limits<double>::infinity();
        double bestCongest = -1.0;
        double bestDist = std::numeric_limits<double>::infinity();
        int bestNode = -1;
        for (int nodeId : candidates) {
            if (nodeId < 0) continue;
            try {
                const Node& node = mapInfo.getNodeById(nodeId);
                consider_node(nodeId,
                              node.x,
                              node.y,
                              requireLow,
                              requireDifferentRegion,
                              requireNearGoal,
                              bestScore,
                              bestCongest,
                              bestDist,
                              bestNode);
            } catch (...) {
                continue;
            }
        }
        return bestNode;
    };

    auto pick_with_pref = [&](const auto& picker) -> int {
        if (useRegion) {
            int chosen = picker(true, true);
            if (chosen >= 0) return chosen;
            chosen = picker(true, false);
            if (chosen >= 0) return chosen;
            chosen = picker(false, true);
            if (chosen >= 0) return chosen;
            return picker(false, false);
        }
        int chosen = picker(true, false);
        if (chosen >= 0) return chosen;
        return picker(false, false);
    };

    if (const char* env = std::getenv("TEMP_GOAL_CANDIDATES")) {
        auto candidates = parse_env_node_list(env);
        if (!candidates.empty()) {
            int chosen = pick_with_pref([&](bool requireLow, bool requireDifferentRegion) {
                return pick_from_list(candidates, requireLow, requireDifferentRegion, false);
            });
            if (chosen >= 0) return chosen;
        }
    }

    auto pick_from_nodes = [&](bool requireLow, bool requireDifferentRegion, bool requireNearGoal) -> int {
        double bestScore = std::numeric_limits<double>::infinity();
        double bestCongest = -1.0;
        double bestDist = std::numeric_limits<double>::infinity();
        int bestNode = -1;
        for (const auto& node : nodes) {
            if (node.type == -1) continue;
            if (node.allowPass == 0) continue;
            consider_node(node.id,
                          node.x,
                          node.y,
                          requireLow,
                          requireDifferentRegion,
                          requireNearGoal,
                          bestScore,
                          bestCongest,
                          bestDist,
                          bestNode);
        }
        return bestNode;
    };

    int bestNode = -1;
    if (hasGoal && nearGoalMm > 0.0) {
        bestNode = pick_with_pref([&](bool requireLow, bool requireDifferentRegion) {
            return pick_from_nodes(requireLow, requireDifferentRegion, true);
        });
    }
    if (bestNode < 0) {
        bestNode = pick_with_pref([&](bool requireLow, bool requireDifferentRegion) {
            return pick_from_nodes(requireLow, requireDifferentRegion, false);
        });
    }
    if (bestNode < 0) return std::nullopt;
    return bestNode;
}

static PathPlanningHelper::AmrPlanInfo compute_dynamic_plan_to_next_target(
    const std::string& deviceId,
    int startNodeId,
    int committedNextNodeId,
    int batteryLevel,
    int deviceType,
    const PlanCacheEntry& planEntry,
    RobotDataRepository& repo,
    const MapInfo& mapInfo,
    bool skipStaticTable,
    bool staticTableReady,
    StaticPathTable& staticTable,
    AStarPathFinder& aStar,
    NodeReservationTable& reservations,
    int reserveBudgetOverride,
    bool allowLongerRoute,
    ReplanStage stage,
    bool* updatedOut,
    bool* usedTempGoalOut) {
    PathPlanningHelper::AmrPlanInfo out;
    out.amrId = deviceId;
    out.startNodeId = startNodeId;
    if (updatedOut) *updatedOut = true;
    if (usedTempGoalOut) *usedTempGoalOut = false;
    const bool logSummary = env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL");
    const bool logDetail = env_enabled("RECEIVER_LOG_ROUTE_DETAIL");
    const bool profileDetail = env_enabled("REPLAN_PROFILE_DETAIL");
    struct PlanProfile {
        long long congestionMs = 0;
        long long regionMs = 0;
        long long staticMs = 0;
        long long astarMs = 0;
        int astarCalls = 0;
        long long reserveMs = 0;
    };
    PlanProfile profile;
    struct ProfileGuard {
        bool enabled = false;
        std::string deviceId;
        std::chrono::steady_clock::time_point start{};
        PlanProfile* profile = nullptr;
        ~ProfileGuard() {
            if (!enabled || !profile) return;
            const auto end = std::chrono::steady_clock::now();
            const auto totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << log_time_prefix()
                      << "[ReplanProfile] deviceId=" << deviceId
                      << " totalMs=" << totalMs
                      << " congestionMs=" << profile->congestionMs
                      << " regionMs=" << profile->regionMs
                      << " staticMs=" << profile->staticMs
                      << " astarMs=" << profile->astarMs
                      << " astarCalls=" << profile->astarCalls
                      << " reserveMs=" << profile->reserveMs
                      << std::endl;
        }
    };
    ProfileGuard profileGuard{profileDetail, deviceId, std::chrono::steady_clock::now(), &profile};
    auto mark_ms = [&](long long& acc, const std::chrono::steady_clock::time_point& start) {
        if (!profileDetail) return;
        const auto end = std::chrono::steady_clock::now();
        acc += std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    };

    if (deviceId.empty()) {
        out.allReachable = false;
        return out;
    }

    maybe_cleanup_reservations(reservations);
    maybe_release_stale_reservations(repo, reservations);

    PlanRuntimeState rt = repo.snapshotRuntimeState(deviceId);

    int trailReserveMin = std::max(1, getenv_int("TRAIL_MAX_POINTS", 10));
    if (reserveBudgetOverride <= 0) {
        reserveBudgetOverride = trailReserveMin;
    } else if (reserveBudgetOverride < trailReserveMin) {
        reserveBudgetOverride = trailReserveMin;
    }

    // Align committed window with last sent trail to prevent rollback on replans.
    // But keep detour (temp goal / SUPER) reservations intact so they can reach TrailResp.
    std::vector<int> committedPrefix = repo.getLastSentRoute(deviceId);
    if (!committedPrefix.empty()) {
        const bool keepDetourReserved = (stage == ReplanStage::SUPER) || (rt.tempGoalNodeId >= 0);
        if (!keepDetourReserved) {
            rt.reservedNodes = committedPrefix;
        } else if (logSummary) {
            std::cout << log_time_prefix()
                      << "[Replan] keep detour reserved deviceId=" << deviceId
                      << " stage=" << replan_stage_label(stage)
                      << " tempGoal=" << rt.tempGoalNodeId
                      << " lastSent=" << format_nodes_head(committedPrefix)
                      << " reserved=" << format_nodes_head(rt.reservedNodes)
                      << std::endl;
        }
    }
    if (!rt.reservedNodes.empty()) {
        auto it = std::find(rt.reservedNodes.begin(), rt.reservedNodes.end(), startNodeId);
        if (it != rt.reservedNodes.end()) {
            if (it != rt.reservedNodes.begin()) {
                rt.reservedNodes.erase(rt.reservedNodes.begin(), it);
            }
        } else {
            if (logSummary) {
                std::cout << log_time_prefix()
                          << "[Replan] start node not in reserved prefix deviceId=" << deviceId
                          << " startNode=" << startNodeId
                          << " reservedHead=" << format_nodes_head(rt.reservedNodes)
                          << " reservedSize=" << rt.reservedNodes.size()
                          << " -> resetCommitted=" << startNodeId
                          << std::endl;
            }
            // Status nodeId is not inside the committed prefix; keeping the old prefix would create
            // phantom committed holds and can block the whole fleet. Reset to the status node.
            rt.reservedNodes.clear();
            committedPrefix.clear();
            if (startNodeId >= 0) {
                rt.reservedNodes.push_back(startNodeId);
                committedPrefix.push_back(startNodeId);
            }
            repo.updateLastSentRoute(deviceId, committedPrefix);
        }
    }
    if (!rt.reservedNodes.empty()) {
        committedPrefix = rt.reservedNodes;
    }
    if (rt.reservedNodes.empty() && startNodeId >= 0) {
        rt.reservedNodes.push_back(startNodeId);
    }
    // Ignore status nextDestinationPoint; only trust the committed prefix window.
    committedNextNodeId = -1;
    if (startNodeId < 0) {
        out.allReachable = false;
        return out;
    }
    out.startNodeId = startNodeId;

    auto statusNowOpt = repo.getStatusById(deviceId);
    const bool enableTaskStatusGate = (getenv_int("REPLAN_TASK_STATUS_GATE_ENABLE", 0) != 0);
    if (enableTaskStatusGate) {
        const int gateWaitMs = std::max(0, getenv_int("REPLAN_TASK_STATUS_GATE_WAIT_MS", 200));
        const auto nowGate = std::chrono::steady_clock::now();
        if (rt.waitTaskStatusClear && rt.waitTaskStatusReadyAt == std::chrono::steady_clock::time_point{}) {
            rt.waitTaskStatusReadyAt = nowGate + std::chrono::milliseconds(gateWaitMs);
            rt.staticSince = std::chrono::steady_clock::time_point{};
            rt.staticNodeId = -1;
        }
        bool shouldSkipByGate = false;
        if (rt.waitTaskStatusClear) {
            if (nowGate < rt.waitTaskStatusReadyAt) {
                shouldSkipByGate = true;
            } else {
                const bool workingTaskNow = statusNowOpt.has_value() && statusNowOpt->taskStatus == 1;
                if (workingTaskNow) {
                    shouldSkipByGate = true;
                } else {
                    rt.waitTaskStatusClear = false;
                    rt.waitTaskStatusReadyAt = std::chrono::steady_clock::time_point{};
                }
            }
        }
        if (shouldSkipByGate) {
            reservations.releaseAllByOwner(deviceId);
            rt.reservedNodes.clear();
            rt.blockedSince = std::chrono::steady_clock::time_point{};
            rt.tempGoalNodeId = -1;
            rt.tempGoalBans.clear();
            rt.tempGoalTabu.clear();
            rt.tempGoalFreezeUntil = std::chrono::steady_clock::time_point{};
            rt.majorFailCount = 0;
            rt.lastMajorAttempt = std::chrono::steady_clock::time_point{};
            rt.stuckSince = std::chrono::steady_clock::time_point{};
            rt.stuckNodeId = -1;
            rt.staticSince = std::chrono::steady_clock::time_point{};
            rt.staticNodeId = -1;
            rt.statusMismatchSince = std::chrono::steady_clock::time_point{};
            rt.statusMismatchNodeId = -1;
            repo.updateLastSentRoute(deviceId, {});
            repo.updateRuntimeState(deviceId, planEntry.pathId, rt);
            if (logSummary) {
                long long remainMs = 0;
                if (rt.waitTaskStatusReadyAt != std::chrono::steady_clock::time_point{}) {
                    remainMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                        rt.waitTaskStatusReadyAt - nowGate).count();
                    if (remainMs < 0) remainMs = 0;
                }
                std::cout << log_time_prefix()
                          << "[Replan] gate hold deviceId=" << deviceId
                          << " taskStatus=" << (statusNowOpt.has_value() ? statusNowOpt->taskStatus : -1)
                          << " waitRemainMs=" << remainMs
                          << " start=" << startNodeId
                          << std::endl;
            }
            PathPlanningHelper::AmrPlanInfo keep;
            keep.amrId = deviceId;
            keep.startNodeId = startNodeId;
            PathPlanningHelper::PathSegmentInfo seg;
            seg.fromNodeId = startNodeId;
            seg.toNodeId = startNodeId;
            seg.nodes.push_back(startNodeId);
            seg.reachable = true;
            seg.source = GlobalPathPlanner::PathSource::PURE_ASTAR;
            seg.stepType = "-1";
            seg.distanceMm = 0.0;
            seg.timeSec = 0.0;
            seg.serviceTimeSec = 0.0;
            seg.description = "dynamic_plan (task_status_gate_hold)";
            keep.segments.push_back(std::move(seg));
            keep.totalDistanceMm = 0.0;
            keep.totalTimeSec = 0.0;
            keep.allReachable = true;
            return keep;
        }
    }

    if (stage == ReplanStage::SUPER) {
        std::vector<int> keepNodes;
        keepNodes.push_back(startNodeId);
        if (!deviceId.empty()) {
            reservations.releaseAllByOwnerExceptSet(deviceId, keepNodes);
            reservations.tryReserve(startNodeId,
                                     deviceId,
                                     NodeReservationTable::HoldReason::WAITING_POINT,
                                     "super_release",
                                     std::chrono::milliseconds(0));
            rt.reservedNodes = keepNodes;
            committedPrefix = keepNodes;
            repo.updateLastSentRoute(deviceId, keepNodes);
            repo.updateReservedNodesAndMismatchOnly(deviceId,
                                                    keepNodes,
                                                    std::chrono::steady_clock::time_point{},
                                                    -1);
            if (logSummary) {
                std::cout << log_time_prefix()
                          << "[Reserve] super release deviceId=" << deviceId
                          << " keepNode=" << startNodeId
                          << std::endl;
            }
        }
    }

    // Pop reached targets.
    bool progressed = false;
    while (!rt.targets.empty() && rt.targets.front().nodeId == startNodeId) {
        rt.targets.pop_front();
        progressed = true;
    }
    if (progressed) {
        rt.blockedSince = std::chrono::steady_clock::time_point{};
        rt.tempGoalNodeId = -1;
        rt.lastFirstHopFromNodeId = -1;
        rt.lastFirstHopNodeId = -1;
        rt.lastFirstHopSince = std::chrono::steady_clock::time_point{};
        rt.tempGoalBans.clear();
        if (enableTaskStatusGate) {
            rt.waitTaskStatusClear = true;
            rt.waitTaskStatusReadyAt = std::chrono::steady_clock::time_point{};
        }
    }

    std::optional<TargetPointEntry> target;
    if (!rt.targets.empty()) target = rt.targets.front();

    int primaryGoal = target ? target->nodeId : -1;
    const bool onOtherTaskNode = is_node_targeted_by_other(repo, deviceId, startNodeId);
    int taskPriority = 0;
    if (target && !target->taskId.empty()) {
        auto it = planEntry.taskPriorities.find(target->taskId);
        if (it != planEntry.taskPriorities.end()) taskPriority = it->second;
    }
    int baseReplanMs = std::max(20, getenv_int("REPLAN_TICK_MS", 200));
    int replanIntervalMs = baseReplanMs;
    double reserveScale = 1.0;
    double goalDistMm = -1.0;
    if (primaryGoal >= 0) {
        goalDistMm = estimate_goal_distance_mm(mapInfo, startNodeId, primaryGoal, &aStar);
    }
    int reserveBudget = compute_reserve_budget_nodes(taskPriority, batteryLevel, deviceType);
    if (reserveBudgetOverride > 0) {
        // Keep reservation >= promised window length.
        reserveBudget = std::max(reserveBudget, reserveBudgetOverride);
    }
    if (reserveScale > 1.0 && reserveBudget > 0) {
        long long scaled = std::llround(static_cast<double>(reserveBudget) * reserveScale);
        int reserveMax = getenv_int("RESERVE_MAX_NODES", 20);
        if (reserveMax > 0) scaled = std::min<long long>(scaled, reserveMax);
        if (scaled > std::numeric_limits<int>::max()) scaled = std::numeric_limits<int>::max();
        reserveBudget = static_cast<int>(scaled);
    }
    if (!rt.reservedNodes.empty() && static_cast<int>(rt.reservedNodes.size()) > reserveBudget) {
        // Never shrink below the already-committed prefix to avoid rollback.
        reserveBudget = static_cast<int>(rt.reservedNodes.size());
    }
    // Default: only reserve as many nodes as we publish in the trail window.
    // Reserving far beyond the published trail creates "phantom blocks" (others get blocked by
    // nodes the controller has not even received yet).
    if (getenv_int("RESERVE_MATCH_TRAIL_POINTS", 1) != 0) {
        reserveBudget = std::min(reserveBudget, reserveBudgetOverride);
        if (!rt.reservedNodes.empty()) {
            reserveBudget = std::max(reserveBudget, static_cast<int>(rt.reservedNodes.size()));
        }
    }
    if (reserveBudget <= 0) reserveBudget = 1;
    const int holdTtlMs = getenv_int("RESERVE_TTL_MS", 15000);
    const int committedTtlMs = std::max(0, getenv_int("RESERVE_COMMITTED_TTL_MS", 0));

    // Two planning modes:
    // - realtime replan (allowLongerRoute==false): ignore held nodes in A* (plan first, then truncate on reserve),
    //   but still honor congestion/region hard-block if enabled.
    // - major replan (allowLongerRoute==true): treat held nodes as hard obstacles to seek a feasible detour.
    const bool avoidHeldNodesInPlanning = allowLongerRoute;

    // Build blocked nodes snapshot (other AGVs, expanded by near-conflict rule). We only inject them as A* banned nodes when
    // avoidHeldNodesInPlanning==true; otherwise we plan freely and rely on non-preemptive reserve.
    auto blockedVec = reservations.snapshotBlockedNodes(deviceId);
    std::unordered_set<int> bannedNodes;
    bannedNodes.reserve(blockedVec.size());
    std::unordered_set<int> bannedNodesAll;
    bannedNodesAll.reserve(blockedVec.size());
    for (int nodeId : blockedVec) {
        bannedNodesAll.insert(nodeId);
        if (avoidHeldNodesInPlanning) {
            bannedNodes.insert(nodeId);
        }
    }

    bool statusStaticNow = false;
    long long statusStaticMs = 0;
    const auto now = std::chrono::steady_clock::now();
    const int firstHopCooldownMs = std::max(0, getenv_int("REPLAN_FIRST_HOP_COOLDOWN_MS", 2000));
    const bool firstHopEnabled = (getenv_int("REPLAN_FIRST_HOP_ENABLE", 0) != 0);
    const int firstHopWaitMs = std::max(0, getenv_int("REPLAN_FIRST_HOP_WAIT_MS", 0));
    const int tempGoalBanMs = getenv_int("REPLAN_TEMP_GOAL_BAN_MS", 10000);
    if (tempGoalBanMs > 0) {
        const auto banWindow = std::chrono::milliseconds(tempGoalBanMs);
        for (auto it = rt.tempGoalBans.begin(); it != rt.tempGoalBans.end();) {
            if (now - it->second >= banWindow) {
                it = rt.tempGoalBans.erase(it);
            } else {
                ++it;
            }
        }
    } else {
        rt.tempGoalBans.clear();
    }
    const int tempGoalFreezeMs = std::max(0, getenv_int("TEMP_GOAL_FREEZE_MS", 5000));
    const int tempGoalTabuSize = std::max(0, getenv_int("TEMP_GOAL_TABU_SIZE", 5));
    if (tempGoalTabuSize <= 0) {
        rt.tempGoalTabu.clear();
    } else {
        while (static_cast<int>(rt.tempGoalTabu.size()) > tempGoalTabuSize) {
            rt.tempGoalTabu.pop_front();
        }
    }
    if (rt.tempGoalFreezeUntil != std::chrono::steady_clock::time_point{} &&
        now >= rt.tempGoalFreezeUntil) {
        rt.tempGoalFreezeUntil = std::chrono::steady_clock::time_point{};
    }
    auto push_temp_goal_tabu = [&](int nodeId) {
        if (nodeId < 0 || tempGoalTabuSize <= 0) return;
        auto it = std::find(rt.tempGoalTabu.begin(), rt.tempGoalTabu.end(), nodeId);
        if (it != rt.tempGoalTabu.end()) {
            rt.tempGoalTabu.erase(it);
        }
        rt.tempGoalTabu.push_back(nodeId);
        while (static_cast<int>(rt.tempGoalTabu.size()) > tempGoalTabuSize) {
            rt.tempGoalTabu.pop_front();
        }
    };
    auto mark_temp_goal_reached = [&]() -> bool {
        if (rt.tempGoalNodeId < 0 || rt.tempGoalNodeId != startNodeId) return false;
        const int reached = rt.tempGoalNodeId;
        rt.tempGoalNodeId = -1;
        push_temp_goal_tabu(reached);
        if (tempGoalFreezeMs > 0) {
            rt.tempGoalFreezeUntil = now + std::chrono::milliseconds(tempGoalFreezeMs);
        }
        if (logSummary && tempGoalFreezeMs > 0) {
            std::cout << log_time_prefix()
                      << "[TempGoal] reached deviceId=" << deviceId
                      << " node=" << reached
                      << " freezeMs=" << tempGoalFreezeMs
                      << std::endl;
        }
        return true;
    };
    mark_temp_goal_reached();
    auto build_plan_from_reserved = [&](const std::vector<int>& reserved) -> PathPlanningHelper::AmrPlanInfo {
        PathPlanningHelper::AmrPlanInfo keep;
        keep.amrId = deviceId;
        std::vector<int> nodes = reserved;
        if (nodes.empty() && startNodeId >= 0) nodes.push_back(startNodeId);
        if (!nodes.empty() && startNodeId >= 0) {
            auto it = std::find(nodes.begin(), nodes.end(), startNodeId);
            if (it != nodes.end()) {
                if (it != nodes.begin()) nodes.erase(nodes.begin(), it);
            } else {
                nodes.insert(nodes.begin(), startNodeId);
            }
        }
        if (nodes.empty() && startNodeId >= 0) nodes.push_back(startNodeId);
        PathPlanningHelper::PathSegmentInfo seg;
        seg.nodes = std::move(nodes);
        if (!seg.nodes.empty()) {
            seg.fromNodeId = seg.nodes.front();
            seg.toNodeId = seg.nodes.back();
        } else {
            seg.fromNodeId = startNodeId;
            seg.toNodeId = startNodeId;
        }
        seg.reachable = true;
        seg.source = GlobalPathPlanner::PathSource::PURE_ASTAR;
        if (target) {
            seg.taskId = target->taskId;
            seg.subTaskId = target->subTaskId;
            seg.subTaskSequence = target->subTaskSequence;
            seg.stepType = (seg.toNodeId == primaryGoal) ? target->stepType : std::string("-1");
        } else {
            seg.taskId.clear();
            seg.subTaskId.clear();
            seg.subTaskSequence = -1;
            seg.stepType = "-1";
        }
        seg.distanceMm = compute_route_distance_mm(mapInfo, seg.nodes);
        double speed = mapInfo.getGlobalMaxSpeed();
        if (!(speed > 0.0)) speed = 1000.0;
        seg.timeSec = seg.distanceMm / speed;
        seg.serviceTimeSec = 0.0;
        seg.description = "dynamic_plan (keep_old)";
        keep.segments.push_back(std::move(seg));
        if (!keep.segments.empty()) {
            keep.startNodeId = keep.segments.front().fromNodeId;
            keep.totalDistanceMm = keep.segments.front().distanceMm;
            keep.totalTimeSec = keep.segments.front().timeSec;
        } else {
            keep.startNodeId = startNodeId;
            keep.totalDistanceMm = 0.0;
            keep.totalTimeSec = 0.0;
        }
        keep.allReachable = true;
        return keep;
    };
    bool tempGoalFrozen = false;
    if (tempGoalFreezeMs > 0 &&
        rt.tempGoalFreezeUntil != std::chrono::steady_clock::time_point{} &&
        now < rt.tempGoalFreezeUntil) {
        tempGoalFrozen = true;
    }
    if (tempGoalFrozen) {
        if (startNodeId >= 0 && !deviceId.empty()) {
            std::vector<int> keepNodes;
            keepNodes.push_back(startNodeId);
            reservations.releaseAllByOwnerExceptSet(deviceId, keepNodes);
            rt.reservedNodes = keepNodes;
            committedPrefix = keepNodes;
            repo.updateLastSentRoute(deviceId, keepNodes);
            repo.updateReservedNodesAndMismatchOnly(deviceId,
                                                    keepNodes,
                                                    std::chrono::steady_clock::time_point{},
                                                    -1);
            if (logSummary) {
                long long remainMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    rt.tempGoalFreezeUntil - now).count();
                if (remainMs < 0) remainMs = 0;
                std::cout << log_time_prefix()
                          << "[TempGoal] freeze deviceId=" << deviceId
                          << " remainMs=" << remainMs
                          << " keepNode=" << startNodeId
                          << std::endl;
            }
        }
        if (updatedOut) *updatedOut = false;
        repo.updateRuntimeState(deviceId, planEntry.pathId, rt);
        return build_plan_from_reserved(rt.reservedNodes);
    }
    if (stage == ReplanStage::MAJOR || stage == ReplanStage::SUPER) {
        rt.lastMajorAttempt = now;
    }
    const int staticPosEpsMm = getenv_int("STATUS_STATIC_POS_EPS_MM", 10);
    const double staticSpeedEps = getenv_double("STATUS_STATIC_SPEED_EPS", 0.01);
    const double staticYawEps = getenv_double("STATUS_STATIC_YAW_EPS_DEG", -1.0);
    const bool hasGoal = (primaryGoal >= 0) || (rt.tempGoalNodeId >= 0);
    const bool hasTrajectoryWindow =
        (rt.reservedNodes.size() > 1) || (committedPrefix.size() > 1);
    std::optional<RobotStatusEntry> statusOpt;
    if (startNodeId >= 0 && staticPosEpsMm >= 0 && staticSpeedEps >= 0.0) {
        statusOpt = repo.getStatusById(deviceId);
        if (statusOpt.has_value()) {
            const auto& st = *statusOpt;
            const bool hasPrev = (rt.lastStatusSeen != std::chrono::steady_clock::time_point{});
            const bool speedStatic = std::abs(st.speed) <= staticSpeedEps;
            const bool nodeStatic = hasPrev && (rt.lastStatusNodeId == startNodeId);
            const bool posStatic = hasPrev &&
                (std::abs(st.x - rt.lastStatusX) <= staticPosEpsMm) &&
                (std::abs(st.y - rt.lastStatusY) <= staticPosEpsMm);
            bool yawStatic = true;
            if (staticYawEps >= 0.0 && hasPrev) {
                double diff = std::abs(st.angle - rt.lastStatusAngle);
                diff = std::fmod(diff, 360.0);
                if (diff > 180.0) diff = 360.0 - diff;
                yawStatic = diff <= staticYawEps;
            }
            if (hasPrev && nodeStatic && posStatic && speedStatic && yawStatic && hasGoal &&
                !hasTrajectoryWindow) {
                if (rt.staticSince == std::chrono::steady_clock::time_point{} ||
                    rt.staticNodeId != startNodeId) {
                    rt.staticSince = now;
                    rt.staticNodeId = startNodeId;
                }
                statusStaticNow = true;
                statusStaticMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - rt.staticSince).count();
            } else {
                rt.staticSince = std::chrono::steady_clock::time_point{};
                rt.staticNodeId = -1;
            }
            rt.lastStatusX = st.x;
            rt.lastStatusY = st.y;
            rt.lastStatusAngle = st.angle;
            rt.lastStatusNodeId = startNodeId;
            rt.lastStatusSeen = now;
        } else {
            rt.staticSince = std::chrono::steady_clock::time_point{};
            rt.staticNodeId = -1;
            rt.lastStatusSeen = std::chrono::steady_clock::time_point{};
        }
    } else {
        rt.staticSince = std::chrono::steady_clock::time_point{};
        rt.staticNodeId = -1;
        rt.lastStatusSeen = std::chrono::steady_clock::time_point{};
    }
    const int staticReleaseMs = getenv_int("STATUS_STATIC_RELEASE_MS", -1);
    if (staticReleaseMs >= 0 && statusStaticNow && statusStaticMs >= staticReleaseMs) {
        std::vector<int> keepNodes;
        if (startNodeId >= 0) {
            keepNodes.push_back(startNodeId);
        }
        std::vector<int> beforeReserved = rt.reservedNodes;
        std::vector<int> beforeCommitted = repo.getLastSentRoute(deviceId);
        bool needRelease = false;
        if (!keepNodes.empty()) {
            if (beforeReserved.size() > keepNodes.size()) {
                needRelease = true;
            } else if (!beforeReserved.empty() && beforeReserved.front() != keepNodes.front()) {
                needRelease = true;
            }
            if (!beforeCommitted.empty()) {
                needRelease = true;
            }
        } else if (!beforeReserved.empty() || !beforeCommitted.empty()) {
            needRelease = true;
        }
        if (needRelease) {
            std::cout << log_time_prefix()
                      << "[Reserve] static release deviceId=" << deviceId
                      << " staticMs=" << statusStaticMs
                      << " thresholdMs=" << staticReleaseMs
                      << " start=" << startNodeId
                      << " keep=" << format_nodes_full(keepNodes)
                      << " reservedBefore=" << format_nodes_full(beforeReserved)
                      << " committedBefore=" << format_nodes_full(beforeCommitted)
                      << " tempGoal=" << rt.tempGoalNodeId
                      << " posEps=" << staticPosEpsMm
                      << " speedEps=" << staticSpeedEps
                      << " yawEps=" << staticYawEps
                      << std::endl;
            if (statusOpt.has_value()) {
                const auto& st = *statusOpt;
                std::cout << log_time_prefix()
                          << "[Reserve] static release status deviceId=" << deviceId
                          << " taskStatus=" << st.taskStatus
                          << " taskId=" << (st.taskId.empty() ? "<empty>" : st.taskId)
                          << " speed=" << st.speed
                          << " nodeId=" << (st.nodeId.empty() ? "<empty>" : st.nodeId)
                          << " x=" << static_cast<int>(std::lround(st.x))
                          << " y=" << static_cast<int>(std::lround(st.y))
                          << " angle=" << st.angle
                          << " updateTime=" << st.updateTime
                          << " mapId=" << st.mapId
                          << std::endl;
            } else {
                std::cout << log_time_prefix()
                          << "[Reserve] static release status deviceId=" << deviceId
                          << " status=<none>"
                          << std::endl;
            }
            if (!keepNodes.empty()) {
                reservations.releaseAllByOwnerExceptSet(deviceId, keepNodes);
                reservations.tryReserve(startNodeId,
                                         deviceId,
                                         NodeReservationTable::HoldReason::WAITING_POINT,
                                         "static_release",
                                         std::chrono::milliseconds(0));
            } else {
                reservations.releaseAllByOwner(deviceId);
            }
            rt.reservedNodes = keepNodes;
            rt.statusMismatchSince = std::chrono::steady_clock::time_point{};
            rt.statusMismatchNodeId = -1;
            repo.updateReservedNodesAndMismatchOnly(deviceId,
                                                    keepNodes,
                                                    rt.statusMismatchSince,
                                                    rt.statusMismatchNodeId);
            repo.updateLastSentRoute(deviceId, {});
            // Reset static timer after releasing reservations to avoid immediate repeated releases.
            rt.staticSince = now;
            rt.staticNodeId = startNodeId;
            std::cout << log_time_prefix()
                      << "[Reserve] static release done deviceId=" << deviceId
                      << " reservedAfter=" << format_nodes_full(keepNodes)
                      << " committedAfter=<empty>"
                      << std::endl;
        }
    }
    if (!statusStaticNow && rt.majorFailCount != 0) {
        rt.majorFailCount = 0;
        rt.lastMajorAttempt = std::chrono::steady_clock::time_point{};
    }
    const bool enableCongestionAvoid = (getenv_int("CONGESTION_AVOID", 1) != 0);
    // Fast replans should not consider other AGVs' reservations during planning (soft congestion included).
    std::vector<int> emptyBlocked;
    const std::vector<int>& congestionBlocked = avoidHeldNodesInPlanning ? blockedVec : emptyBlocked;
    CongestionInfo congestion;
    {
        const auto t0 = std::chrono::steady_clock::now();
        congestion = build_congestion_info(mapInfo, reservations, congestionBlocked, now);
        mark_ms(profile.congestionMs, t0);
    }
    RegionCongestionInfo region;
    if (enableCongestionAvoid) {
        const auto t0 = std::chrono::steady_clock::now();
        auto statusSnapshot = repo.snapshotStatuses();
        region = build_region_congestion_info(mapInfo, statusSnapshot);
        merge_region_congestion(region, congestion);
        mark_ms(profile.regionMs, t0);
    }
    std::unordered_set<int> congestionNodes = congestion.hardBlocked;
    congestionNodes.insert(congestion.hotNodes.begin(), congestion.hotNodes.end());
    const double tempLowScore = std::max(0.0, getenv_double("CONGESTION_TEMP_LOW_SCORE", 0.0));
    const double aStarPenaltyMm = std::max(0.0, getenv_double("CONGESTION_ASTAR_PENALTY_MM", 2000.0));
    const int fastHeldPenaltyMs = std::max(0, getenv_int("REPLAN_FAST_HELD_PENALTY_MS", 1000));
    std::unordered_map<int, double> fastHeldPenaltyScore;
    double fastHeldPenaltyMm = 0.0;
    if (stage == ReplanStage::FAST && fastHeldPenaltyMs > 0 && !blockedVec.empty()) {
        double speed = mapInfo.getGlobalMaxSpeed();
        if (!(speed > 0.0)) speed = 1000.0;
        fastHeldPenaltyMm = speed * (static_cast<double>(fastHeldPenaltyMs) / 1000.0);
        fastHeldPenaltyScore.reserve(blockedVec.size());
        for (int nodeId : blockedVec) {
            fastHeldPenaltyScore[nodeId] = 1.0;
        }
    }
    const bool stuckHere = false;
    if (logSummary) {
        std::cout << log_time_prefix()
                  << "[Replan] ctx deviceId=" << deviceId
                  << " start=" << startNodeId
                  << " goal=" << primaryGoal
                  << " allowLonger=" << (allowLongerRoute ? "1" : "0")
                  << " reserveBudget=" << reserveBudget
                  << " trailMin=" << trailReserveMin
                  << " reservedLen=" << rt.reservedNodes.size()
                  << " committedLen=" << committedPrefix.size()
                  << " goalDistMm=" << std::llround(goalDistMm)
                  << " replanMs=" << replanIntervalMs
                  << " staticMs=" << statusStaticMs
                  << " avoidHeld=" << (avoidHeldNodesInPlanning ? "1" : "0")
                  << " heldCount=" << bannedNodesAll.size()
                  << " bannedCount=" << bannedNodes.size()
                  << " congHard=" << congestion.hardBlocked.size()
                  << " congHot=" << congestion.hotNodes.size()
                  << " regionHard=" << region.hardBlockedNodes.size()
                  << " stuck=" << (stuckHere ? "1" : "0")
                  << std::endl;
    }

    auto is_direct_edge = [&](int fromNodeId, int toNodeId) -> bool {
        if (fromNodeId < 0 || toNodeId < 0) return false;
        const auto& id2idx = mapInfo.getId2Index();
        auto itFrom = id2idx.find(fromNodeId);
        auto itTo = id2idx.find(toNodeId);
        if (itFrom == id2idx.end() || itTo == id2idx.end()) return false;
        const int fromIdx = itFrom->second;
        const int toIdx = itTo->second;
        const auto& aft = mapInfo.getAftNode();
        auto it = aft.find(fromIdx);
        if (it == aft.end()) return false;
        const auto& v = it->second;
        return std::find(v.begin(), v.end(), toIdx) != v.end();
    };

    auto plan_from_to = [&](int fromNodeId,
                            int goalNodeId,
                            std::vector<int>& routeOut,
                            bool& usedStatic,
                            const std::unordered_set<int>* overrideBanned) -> bool {
        routeOut.clear();
        usedStatic = false;
        if (fromNodeId < 0 || goalNodeId < 0) return false;
        const std::unordered_set<int>* baseBanned = overrideBanned ? overrideBanned : &bannedNodes;
        std::unordered_set<int> localBanned;
        const bool goalHeldByOther = (goalNodeId >= 0 && baseBanned && baseBanned->count(goalNodeId) > 0);
        if (goalHeldByOther && goalNodeId != fromNodeId && (avoidHeldNodesInPlanning || overrideBanned)) {
            return false;
        }
	        std::vector<int> hardBlockedFiltered;
        if (enableCongestionAvoid && !congestion.hardBlocked.empty()) {
            hardBlockedFiltered = filter_hard_blocked_nodes_for_start(congestion.hardBlocked, region, mapInfo, reservations, fromNodeId);
            localBanned.insert(hardBlockedFiltered.begin(), hardBlockedFiltered.end());
        }
        if (baseBanned && !baseBanned->empty()) {
            localBanned.insert(baseBanned->begin(), baseBanned->end());
        }
	        if (!localBanned.empty()) {
	            localBanned.erase(fromNodeId);
            bool goalBlockedByRegion = false;
            if (enableCongestionAvoid && region.valid() && !region.hardBlockedNodes.empty()) {
                int startCell = region.cellIndexForNode(mapInfo, fromNodeId);
                int goalCell = region.cellIndexForNode(mapInfo, goalNodeId);
                if (region.hardBlockedNodes.count(goalNodeId) > 0 &&
                    startCell >= 0 && goalCell >= 0 && startCell != goalCell) {
                    goalBlockedByRegion = true;
                }
            }
            if (!goalBlockedByRegion && !goalHeldByOther) {
                localBanned.erase(goalNodeId);
            }
        }
	        const std::unordered_set<int>* bannedPtr = localBanned.empty() ? nullptr : &localBanned;

        if (!skipStaticTable && staticTableReady) {
            StaticPathTable::DynamicContext ctx;
            if (baseBanned && !baseBanned->empty()) {
                ctx.blockedNodes.insert(baseBanned->begin(), baseBanned->end());
            }
            if (enableCongestionAvoid && !hardBlockedFiltered.empty()) {
                ctx.blockedNodes.insert(hardBlockedFiltered.begin(), hardBlockedFiltered.end());
            }
	            if (enableCongestionAvoid && !congestion.nodeScore.empty()) {
	                for (const auto& kv : congestion.nodeScore) {
	                    if (kv.second <= 0.0) continue;
	                    double v = std::min(1.0, kv.second);
	                    ctx.nodeCongestion[kv.first] = v;
	                }
	                ctx.avoidCongestion = true;
	            }
	            ctx.blockedNodes.erase(fromNodeId);
            bool goalBlockedByRegion = false;
            if (enableCongestionAvoid && region.valid() && !region.hardBlockedNodes.empty()) {
                int startCell = region.cellIndexForNode(mapInfo, fromNodeId);
                int goalCell = region.cellIndexForNode(mapInfo, goalNodeId);
                if (region.hardBlockedNodes.count(goalNodeId) > 0 &&
                    startCell >= 0 && goalCell >= 0 && startCell != goalCell) {
                    goalBlockedByRegion = true;
                }
	            }
            if (!goalBlockedByRegion && !goalHeldByOther) {
                ctx.blockedNodes.erase(goalNodeId);
            }
            const auto t0 = std::chrono::steady_clock::now();
            auto cand = staticTable.query(fromNodeId, goalNodeId, ctx);
            mark_ms(profile.staticMs, t0);
            if (cand.has_value() && !cand->fullPath.empty()) {
                routeOut = cand->fullPath;
                usedStatic = true;
            }
        }
        if (routeOut.empty()) {
            const std::unordered_map<int, double>* scorePtr =
                (enableCongestionAvoid && !congestion.nodeScore.empty()) ? &congestion.nodeScore : nullptr;
            double penaltyMm = aStarPenaltyMm;
            if (!fastHeldPenaltyScore.empty()) {
                scorePtr = &fastHeldPenaltyScore;
                penaltyMm = fastHeldPenaltyMm;
            }
            const auto t0 = std::chrono::steady_clock::now();
            auto res = aStar.findPath(fromNodeId,
                                      goalNodeId,
                                      PathPlanningConstants::resolveTurnPenaltyMm(),
                                      bannedPtr,
                                      nullptr,
                                      scorePtr,
                                      penaltyMm);
            if (profileDetail) profile.astarCalls += 1;
            mark_ms(profile.astarMs, t0);
            if (res.found && !res.path.empty()) {
                routeOut = res.path;
            }
        }
        if (routeOut.empty()) return false;
        if (routeOut.front() != fromNodeId) {
            routeOut.insert(routeOut.begin(), fromNodeId);
        }
        return true;
    };

    auto plan_to_goal = [&](int goalNodeId,
                            std::vector<int>& routeOut,
                            bool& usedStatic) -> bool {
        return plan_from_to(startNodeId, goalNodeId, routeOut, usedStatic, nullptr);
    };

    auto estimate_route_to_goal_mm = [&](const std::vector<int>& route, int goalNodeId) -> double {
        if (route.empty()) return -1.0;
        double dist = compute_route_distance_mm(mapInfo, route);
        if (goalNodeId >= 0) {
            int tailStart = route.back();
            if (tailStart != goalNodeId) {
                double tail = estimate_goal_distance_mm(mapInfo, tailStart, goalNodeId, &aStar);
                if (tail < 0.0) return -1.0;
                dist += tail;
            }
        }
        return dist;
    };

    if (primaryGoal < 0 && rt.tempGoalNodeId < 0) {
        std::vector<int> keepNodes;
        if (startNodeId >= 0) keepNodes.push_back(startNodeId);
        if (!deviceId.empty()) {
            if (!keepNodes.empty()) {
                reservations.releaseAllByOwnerExceptSet(deviceId, keepNodes);
            } else {
                reservations.releaseAllByOwner(deviceId);
            }
            rt.reservedNodes = keepNodes;
            committedPrefix = keepNodes;
            repo.updateLastSentRoute(deviceId, keepNodes);
        }
        repo.updateRuntimeState(deviceId, planEntry.pathId, rt);
        return build_plan_from_reserved(rt.reservedNodes);
    }

    std::vector<int> plannedRoute;
    bool usedStatic = false;
    bool usingTempGoal = false;
    int selectedGoal = primaryGoal;

    bool allowTempSelect = (stage == ReplanStage::SUPER);
    bool allowTempUse = allowTempSelect || (rt.tempGoalNodeId >= 0);
    if (tempGoalFrozen) {
        allowTempSelect = false;
        allowTempUse = false;
    }

    auto ban_temp_goal = [&](int nodeId) {
        if (nodeId < 0 || tempGoalBanMs <= 0) return;
        rt.tempGoalBans[nodeId] = now;
    };

    auto try_plan_temp_goal = [&](std::unordered_set<int>& tempBanned, bool allowSelectNew) -> bool {
        if (tempGoalBanMs > 0 && !rt.tempGoalBans.empty()) {
            for (const auto& kv : rt.tempGoalBans) {
                tempBanned.insert(kv.first);
            }
        }
        if (tempGoalTabuSize > 0 && !rt.tempGoalTabu.empty()) {
            for (int nodeId : rt.tempGoalTabu) {
                tempBanned.insert(nodeId);
            }
        }
        while (true) {
            if (rt.tempGoalNodeId >= 0 && tempGoalBanMs > 0 &&
                rt.tempGoalBans.count(rt.tempGoalNodeId) > 0) {
                rt.tempGoalNodeId = -1;
            }
            if (rt.tempGoalNodeId >= 0 && rt.tempGoalNodeId == startNodeId) {
                const int reached = rt.tempGoalNodeId;
                tempBanned.insert(reached);
                ban_temp_goal(reached);
                mark_temp_goal_reached();
            }
            if (rt.tempGoalNodeId < 0) {
                if (!allowSelectNew) return false;
                auto tmp = select_temp_goal_node(mapInfo,
                                                 startNodeId,
                                                 tempBanned,
                                                 congestionNodes,
                                                 enableCongestionAvoid ? &congestion.nodeScore : nullptr,
                                                 tempLowScore,
                                                 enableCongestionAvoid ? &region : nullptr,
                                                 primaryGoal);
                if (!tmp.has_value()) return false;
                rt.tempGoalNodeId = *tmp;
                if (logSummary) {
                    std::cout << log_time_prefix()
                              << "[TempGoal] pick deviceId=" << deviceId
                              << " start=" << startNodeId
                              << " temp=" << rt.tempGoalNodeId
                              << " banned=" << tempBanned.size()
                              << " congHard=" << congestion.hardBlocked.size()
                              << " regionHard=" << region.hardBlockedNodes.size()
                              << std::endl;
                }
            }
            if (rt.tempGoalNodeId >= 0) {
                std::vector<int> tmpRoute;
                bool tmpStatic = false;
                // Temp-goal planning should also respect other AGVs' reserved/committed nodes.
                if (plan_from_to(startNodeId, rt.tempGoalNodeId, tmpRoute, tmpStatic, &bannedNodesAll)) {
                    plannedRoute = std::move(tmpRoute);
                    usedStatic = tmpStatic;
                    usingTempGoal = true;
                    selectedGoal = rt.tempGoalNodeId;
                    return true;
                }
                if (logSummary) {
                    std::cout << log_time_prefix()
                              << "[TempGoal] unreachable deviceId=" << deviceId
                              << " start=" << startNodeId
                              << " temp=" << rt.tempGoalNodeId
                              << " banned=" << bannedNodesAll.size()
                              << std::endl;
                }
                if (!allowSelectNew) {
                    ban_temp_goal(rt.tempGoalNodeId);
                    rt.tempGoalNodeId = -1;
                    return false;
                }
                tempBanned.insert(rt.tempGoalNodeId);
                ban_temp_goal(rt.tempGoalNodeId);
                rt.tempGoalNodeId = -1;
            }
        }
    };

    auto log_unreachable_detail = [&](const char* stageTag) {
        if (primaryGoal < 0) return;
        const bool goalHeld = bannedNodesAll.count(primaryGoal) > 0;
        const bool goalHard = congestion.hardBlocked.count(primaryGoal) > 0;
        const bool goalHot = congestion.hotNodes.count(primaryGoal) > 0;
        const bool goalRegion = region.valid() && region.hardBlockedNodes.count(primaryGoal) > 0;
        long long blockedMs = 0;
        if (rt.blockedSince != std::chrono::steady_clock::time_point{}) {
            blockedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - rt.blockedSince).count();
        }
                if (should_log_replan_unreachable()) {
                    std::cout << log_time_prefix()
                              << "[Replan] unreachable-detail stage=" << stageTag
                              << " mode=" << replan_stage_label(stage)
                              << " deviceId=" << deviceId
                              << " start=" << startNodeId
                              << " goal=" << primaryGoal
                              << " allowLonger=" << (allowLongerRoute ? "1" : "0")
                              << " avoidHeld=" << (avoidHeldNodesInPlanning ? "1" : "0")
                              << " reservedLen=" << rt.reservedNodes.size()
                              << " reserveBudget=" << reserveBudget
                              << " goalDistMm=" << std::llround(goalDistMm)
                              << " tempGoal=" << rt.tempGoalNodeId
                              << " usingTemp=" << (usingTempGoal ? "1" : "0")
                              << " heldGoal=" << (goalHeld ? "1" : "0")
                              << " hardGoal=" << (goalHard ? "1" : "0")
                              << " hotGoal=" << (goalHot ? "1" : "0")
                              << " regionGoal=" << (goalRegion ? "1" : "0")
                              << " heldCount=" << bannedNodesAll.size()
                              << " bannedCount=" << bannedNodes.size()
                              << " congHard=" << congestion.hardBlocked.size()
                              << " congHot=" << congestion.hotNodes.size()
                              << " regionHard=" << region.hardBlockedNodes.size()
                              << " blockedMs=" << blockedMs
                              << " majorFail=" << rt.majorFailCount
                              << " staticMs=" << statusStaticMs
                              << std::endl;
                    std::cout << log_time_prefix()
                              << "[Replan] unreachable-reserved deviceId=" << deviceId
                              << " reservedNodes=" << format_nodes_full(rt.reservedNodes)
                              << " committed=" << format_nodes_full(repo.getLastSentRoute(deviceId))
                              << std::endl;
                    if (goalHeld) {
                        auto entry = reservations.findBlockingHold(primaryGoal, deviceId);
                        if (entry.has_value()) {
                            long long ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - entry->updatedAt).count();
                            std::cout << log_time_prefix()
                                      << "[Replan] unreachable-goal-holder deviceId=" << deviceId
                                      << " goal=" << primaryGoal
                                      << " owner=" << entry->ownerAgvId
                                      << " reason=" << hold_reason_label(entry->reason)
                                      << " ageMs=" << ageMs
                                      << (entry->detail.empty() ? "" : " detail=" + entry->detail)
                                      << std::endl;
                        }
                    }
                    if (!bannedNodesAll.empty()) {
                        std::ostringstream oss;
                        int shown = 0;
                        for (int nodeId : blockedVec) {
                            if (shown >= 3) break;
                            auto entry = reservations.findBlockingHold(nodeId, deviceId);
                            if (!entry.has_value()) continue;
                            if (shown > 0) oss << ", ";
                            oss << nodeId << ":" << entry->ownerAgvId << ":" << hold_reason_label(entry->reason);
                            if (!entry->detail.empty()) {
                                oss << "(" << entry->detail << ")";
                            }
                            shown += 1;
                        }
                        if (shown > 0) {
                            std::cout << log_time_prefix()
                                      << "[Replan] unreachable-held-sample deviceId=" << deviceId
                                      << " totalHeld=" << bannedNodesAll.size()
                                      << " sample=" << oss.str()
                                      << std::endl;
                        }
                    }
                }
            };

    bool reachablePrimary = false;
    bool plannedOk = false;
    if (primaryGoal >= 0) {
        reachablePrimary = plan_to_goal(primaryGoal, plannedRoute, usedStatic);
        plannedOk = reachablePrimary;
    }
    if (!reachablePrimary) {
        bool allowTemp = (primaryGoal >= 0) || onOtherTaskNode;
        if (allowTemp) {
            if (statusStaticNow) {
                if (rt.blockedSince == std::chrono::steady_clock::time_point{}) {
                    rt.blockedSince = now;
                }
            } else {
                rt.blockedSince = std::chrono::steady_clock::time_point{};
            }
            bool canTryTemp = (rt.tempGoalNodeId >= 0);
            if (!canTryTemp) {
                if (!statusStaticNow) {
                    canTryTemp = false;
                } else if (onOtherTaskNode || allowLongerRoute) {
                    canTryTemp = true;
                } else {
                    canTryTemp = false;
                }
            }
            if (canTryTemp && allowTempUse) {
                std::unordered_set<int> tempBanned(bannedNodesAll.begin(), bannedNodesAll.end());
                if (try_plan_temp_goal(tempBanned, allowTempSelect)) {
                    plannedOk = true;
                }
            }
        } else {
            rt.blockedSince = std::chrono::steady_clock::time_point{};
            rt.tempGoalNodeId = -1;
        }
	    } else {
	        rt.blockedSince = std::chrono::steady_clock::time_point{};
	        rt.tempGoalNodeId = -1;
	    }

    if (stage == ReplanStage::MAJOR) {
        if (plannedOk) {
            rt.majorFailCount = 0;
        } else if (primaryGoal >= 0) {
            if (rt.majorFailCount < std::numeric_limits<int>::max()) {
                rt.majorFailCount += 1;
            }
            if (statusStaticNow) {
                rt.staticSince = now;
                rt.staticNodeId = startNodeId;
            }
        }
    } else if (stage == ReplanStage::SUPER) {
        if (plannedOk) {
            rt.majorFailCount = 0;
        } else {
            rt.majorFailCount = 0;
            rt.tempGoalNodeId = -1;
            rt.blockedSince = std::chrono::steady_clock::time_point{};
            rt.stuckSince = std::chrono::steady_clock::time_point{};
            rt.stuckNodeId = -1;
        }
    }

    if (!plannedOk) {
        if (primaryGoal >= 0 && !reachablePrimary) {
            std::ostringstream oss;
            oss << "plan failed deviceId=" << deviceId
                << " start=" << startNodeId
                << " goal=" << primaryGoal
                << " reason=unreachable";
            log_line(LogLevel::INFO, oss.str());
            log_unreachable_detail("primary");
        }
        if (updatedOut) *updatedOut = false;
        repo.updateRuntimeState(deviceId, planEntry.pathId, rt);
        return build_plan_from_reserved(rt.reservedNodes);
    }

        if (plannedRoute.empty()) {
            plannedRoute.push_back(startNodeId);
        }

		    // Enforce the already-committed reserved window as an immutable prefix (append-only semantics).
		    // This is required when the downstream controller cannot rollback issued trail points.
    if (!rt.reservedNodes.empty()) {
        std::vector<int> committedPrefix = rt.reservedNodes;
        if (committedPrefix.front() != startNodeId) {
            auto it = std::find(committedPrefix.begin(), committedPrefix.end(), startNodeId);
            if (it != committedPrefix.end()) {
                committedPrefix.erase(committedPrefix.begin(), it);
            } else {
                if (logSummary) {
                    std::cout << log_time_prefix()
                              << "[Replan] committed prefix reset deviceId=" << deviceId
                              << " startNode=" << startNodeId
                              << " reservedHead=" << format_nodes_head(rt.reservedNodes)
                              << " reservedSize=" << rt.reservedNodes.size()
                              << std::endl;
                }
                committedPrefix.clear();
                committedPrefix.push_back(startNodeId);
            }
        }
        if (committedPrefix.size() >= 2) {
            const bool prefixMismatch = (plannedRoute.size() < committedPrefix.size())
                || !std::equal(committedPrefix.begin(), committedPrefix.end(), plannedRoute.begin());
            if (prefixMismatch) {
		                std::vector<int> tail;
		                bool tailStatic = false;
		                if (selectedGoal >= 0 && plan_from_to(committedPrefix.back(), selectedGoal, tail, tailStatic, nullptr)) {
		                    plannedRoute = committedPrefix;
		                    if (!tail.empty()) {
		                        size_t startIdx = (tail.front() == committedPrefix.back()) ? 1 : 0;
		                        plannedRoute.insert(plannedRoute.end(), tail.begin() + static_cast<long long>(startIdx), tail.end());
		                    }
		                    usedStatic = tailStatic;
		                } else {
		                    plannedRoute = committedPrefix;
		                    usedStatic = false;
                }
            }
        }
    }

    if (usingTempGoal) {
        std::ostringstream oss;
        oss << "[AroundPath] detour deviceId=" << deviceId
            << " start=" << startNodeId
            << " primary=" << primaryGoal
            << " temp=" << selectedGoal
            << " routeLen=" << plannedRoute.size()
            << " static=" << (usedStatic ? "1" : "0");
        log_line(LogLevel::INFO, oss.str());
    } else if (primaryGoal >= 0 && !reachablePrimary) {
        std::ostringstream oss;
        oss << "plan failed deviceId=" << deviceId
            << " start=" << startNodeId
            << " goal=" << primaryGoal
            << " reason=unreachable";
        log_line(LogLevel::INFO, oss.str());
        log_unreachable_detail("post");
    }

    std::unordered_set<int> committedSet(committedPrefix.begin(), committedPrefix.end());
    struct ReserveDebugInfo {
        int blockedNode = -1;
        size_t blockedIndex = 0;
        std::string blockedBy;
        std::string blockedReason;
        std::string blockedDetail;
    };

    auto reserve_route = [&](const std::vector<int>& route,
                             bool tempGoal,
                             ReserveDebugInfo* dbg) -> std::vector<int> {
        if (dbg) *dbg = ReserveDebugInfo{};
        std::vector<int> out;
        out.reserve(std::min<size_t>(route.size(), static_cast<size_t>(reserveBudget)));
        for (size_t i = 0; i < route.size() && (int)out.size() < reserveBudget; ++i) {
            int nodeId = route[i];
            if (nodeId < 0) continue;
            const bool isCommitted = committedSet.count(nodeId) > 0;
            NodeReservationTable::HoldReason holdReason = isCommitted
                ? NodeReservationTable::HoldReason::RESERVED_PATH
                : (tempGoal ? NodeReservationTable::HoldReason::TEMP_GOAL
                            : NodeReservationTable::HoldReason::RESERVED_PATH);
            std::string holdDetail = isCommitted ? "committed" : (tempGoal ? "temp_goal" : "path");
            int ttlMs = isCommitted ? committedTtlMs : holdTtlMs;
            if (ttlMs < 0) ttlMs = 0;
            if (nodeId == startNodeId) {
                ttlMs = 0;
            }
            std::chrono::milliseconds ttl(ttlMs);
            if (!reservations.tryReserve(nodeId, deviceId, holdReason, holdDetail, ttl)) {
                if (dbg) {
                    dbg->blockedNode = nodeId;
                    dbg->blockedIndex = i;
                    auto hold = reservations.findBlockingHold(nodeId, deviceId);
                    if (hold.has_value()) {
                        dbg->blockedBy = hold->ownerAgvId;
                        dbg->blockedReason = hold_reason_label(hold->reason);
                        dbg->blockedDetail = hold->detail;
                    }
                }
                if (logSummary) {
                    auto hold = reservations.findBlockingHold(nodeId, deviceId);
                    std::cout << log_time_prefix()
                              << "[Reserve] blocked deviceId=" << deviceId
                              << " node=" << nodeId
                              << " idx=" << i
                              << " holder=" << (hold.has_value() ? hold->ownerAgvId : "<none>")
                              << " reason=" << (hold.has_value() ? hold_reason_label(hold->reason) : "<none>")
                              << " detail=" << (hold.has_value() ? hold->detail : "<none>")
                              << " routeHead=" << format_nodes_head(route)
                              << std::endl;
                }
                break;
            }
            out.push_back(nodeId);
        }
        if (out.empty()) {
            reservations.tryReserve(startNodeId, deviceId, NodeReservationTable::HoldReason::WAITING_POINT,
                                    "current", std::chrono::milliseconds(0));
            out.push_back(startNodeId);
        }
        return out;
    };
    auto reserve_with_profile = [&](const std::vector<int>& route,
                                    bool tempGoal,
                                    ReserveDebugInfo* dbg) -> std::vector<int> {
        const auto t0 = std::chrono::steady_clock::now();
        auto out = reserve_route(route, tempGoal, dbg);
        mark_ms(profile.reserveMs, t0);
        return out;
    };

    auto log_reserve_result = [&](const char* tag,
                                  const std::vector<int>& reserved,
                                  const ReserveDebugInfo& dbg) {
        if (!logSummary) return;
        if (!logDetail && reserved.size() > 1 && dbg.blockedNode < 0) return;
        std::cout << log_time_prefix()
                  << "[Reserve] " << tag
                  << " deviceId=" << deviceId
                  << " reservedLen=" << reserved.size()
                  << " plannedLen=" << plannedRoute.size()
                  << " committedLen=" << committedPrefix.size()
                  << " tempGoal=" << (usingTempGoal ? "1" : "0")
                  << " blockedNode=" << dbg.blockedNode
                  << " blockedIdx=" << dbg.blockedIndex
                  << " blockedBy=" << (dbg.blockedBy.empty() ? "<none>" : dbg.blockedBy)
                  << " blockedReason=" << (dbg.blockedReason.empty() ? "<none>" : dbg.blockedReason)
                  << " blockedDetail=" << (dbg.blockedDetail.empty() ? "<none>" : dbg.blockedDetail)
                  << std::endl;
    };

    if (!usingTempGoal && primaryGoal >= 0 && getenv_int("REPLAN_NEAR_GOAL_CHECK", 0) != 0) {
        double nearGoalMm = std::max(0.0, getenv_double("REPLAN_NEAR_GOAL_MM", 10000.0));
        if (goalDistMm >= 0.0 && goalDistMm <= nearGoalMm) {
            double oldCost = estimate_route_to_goal_mm(rt.reservedNodes, primaryGoal);
            double newCost = estimate_route_to_goal_mm(plannedRoute, primaryGoal);
            if (oldCost >= 0.0 && newCost >= 0.0 && newCost > oldCost + 1.0) {
                if (logSummary) {
                    std::cout << log_time_prefix()
                              << "[Replan] near-goal keep-old deviceId=" << deviceId
                              << " goal=" << primaryGoal
                              << " goalDistMm=" << std::llround(goalDistMm)
                              << " oldCostMm=" << std::llround(oldCost)
                              << " newCostMm=" << std::llround(newCost)
                              << std::endl;
                }
                if (updatedOut) *updatedOut = false;
                repo.updateRuntimeState(deviceId, planEntry.pathId, rt);
                return build_plan_from_reserved(rt.reservedNodes);
            }
        }
    }

    ReserveDebugInfo reserveDbg;
    std::vector<int> newReserved = reserve_with_profile(plannedRoute, usingTempGoal, &reserveDbg);
    log_reserve_result("initial", newReserved, reserveDbg);
    if (newReserved.size() <= 1 && plannedRoute.size() >= 2 &&
        committedPrefix.size() <= 1 && !avoidHeldNodesInPlanning &&
        !bannedNodesAll.empty() && selectedGoal >= 0) {
        std::vector<int> detour;
        bool detourStatic = false;
        if (plan_from_to(startNodeId, selectedGoal, detour, detourStatic, &bannedNodesAll)) {
            if (logSummary) {
                std::cout << log_time_prefix()
                          << "[Replan] reserved prefix blocked deviceId=" << deviceId
                          << " start=" << startNodeId
                          << " goal=" << selectedGoal
                          << " retry=avoid_held"
                          << std::endl;
            }
            plannedRoute = std::move(detour);
            usedStatic = detourStatic;
            newReserved = reserve_with_profile(plannedRoute, usingTempGoal, &reserveDbg);
            log_reserve_result("retry_avoid_held", newReserved, reserveDbg);
        }
    }
    if (newReserved.size() <= 1 && rt.reservedNodes.size() > 1) {
        std::vector<int> keepRoute = rt.reservedNodes;
        if (keepRoute.front() != startNodeId) {
            auto it = std::find(keepRoute.begin(), keepRoute.end(), startNodeId);
            if (it != keepRoute.end()) {
                keepRoute.erase(keepRoute.begin(), it);
            } else {
                keepRoute.clear();
                keepRoute.push_back(startNodeId);
            }
        }
        if (!keepRoute.empty()) {
            plannedRoute = keepRoute;
            usedStatic = false;
            usingTempGoal = false;
            newReserved = reserve_with_profile(plannedRoute, usingTempGoal, &reserveDbg);
            log_reserve_result("keep_old", newReserved, reserveDbg);
            if (updatedOut) *updatedOut = false;
        }
    }
    const bool firstHopWaitOk = (firstHopWaitMs <= 0) || (statusStaticNow && statusStaticMs >= firstHopWaitMs);
    // If we failed to reserve even the first hop (window shrinks to 1 node), try to pick an
    // alternative neighbor as the next step so the robot can keep moving and break gridlocks.
    if (firstHopEnabled && firstHopWaitOk && newReserved.size() <= 1 && plannedRoute.size() >= 2 &&
        plannedRoute[1] != startNodeId && committedPrefix.size() <= 1 &&
        selectedGoal >= 0) {
        std::vector<int> neighbors;
        gather_neighbors_by_id(mapInfo, startNodeId, neighbors);
        const bool cooldownActive = (firstHopCooldownMs > 0 &&
            rt.lastFirstHopFromNodeId == startNodeId &&
            rt.lastFirstHopNodeId >= 0 &&
            (now - rt.lastFirstHopSince < std::chrono::milliseconds(firstHopCooldownMs)));
        double bestDist = std::numeric_limits<double>::infinity();
        std::vector<int> bestRoute;
        bool bestStatic = false;
        int bestNext = -1;
        auto consider_neighbor = [&](int nb) {
            if (nb < 0 || nb == startNodeId) return;
            auto hold = reservations.findBlockingHold(nb, deviceId);
            if (hold.has_value() && hold->ownerAgvId != deviceId) return;
            std::vector<int> tail;
            bool tailStatic = false;
            // Prefer routes that avoid currently-held nodes, but still allow reaching the goal.
            const std::unordered_set<int>* bannedPref = bannedNodesAll.empty() ? nullptr : &bannedNodesAll;
            if (!plan_from_to(nb, selectedGoal, tail, tailStatic, bannedPref)) {
                if (!plan_from_to(nb, selectedGoal, tail, tailStatic, nullptr)) return;
            }
            if (tail.empty()) return;
            std::vector<int> cand;
            cand.reserve(2 + tail.size());
            cand.push_back(startNodeId);
            cand.push_back(nb);
            size_t startIdx = (tail.front() == nb) ? 1 : 0;
            if (startIdx < tail.size()) {
                cand.insert(cand.end(), tail.begin() + static_cast<long long>(startIdx), tail.end());
            }
            dedup_consecutive_nodes(cand);
            if (cand.size() < 2 || cand[1] == startNodeId) return;
            double dist = compute_route_distance_mm(mapInfo, cand);
            if (dist < 0.0) dist = estimate_goal_distance_mm(mapInfo, nb, selectedGoal, &aStar);
            if (dist < 0.0) dist = 0.0;
            if (dist < bestDist) {
                bestDist = dist;
                bestRoute = std::move(cand);
                bestStatic = tailStatic;
                bestNext = nb;
            }
        };
        if (cooldownActive) {
            consider_neighbor(rt.lastFirstHopNodeId);
        }
        if (bestNext < 0) {
            for (int nb : neighbors) {
                if (cooldownActive && nb == rt.lastFirstHopNodeId) continue;
                consider_neighbor(nb);
            }
        }
        if (bestNext >= 0 && bestRoute.size() >= 2) {
            plannedRoute = std::move(bestRoute);
            usedStatic = bestStatic;
            newReserved = reserve_with_profile(plannedRoute, usingTempGoal, &reserveDbg);
            log_reserve_result("first_hop", newReserved, reserveDbg);
            if (updatedOut) *updatedOut = false;
            if (newReserved.size() >= 2 && newReserved[1] == bestNext) {
                rt.lastFirstHopFromNodeId = startNodeId;
                rt.lastFirstHopNodeId = bestNext;
                rt.lastFirstHopSince = now;
            }
            if (logSummary) {
                std::cout << log_time_prefix()
                          << "[Replan] first-hop reroute deviceId=" << deviceId
                          << " start=" << startNodeId
                          << " goal=" << selectedGoal
                          << " next=" << bestNext
                          << " reservedLen=" << newReserved.size()
                          << std::endl;
            }
        }
    }
    // Release obsolete holds after we have a confirmed reserved window.
    const bool autoCommit = (getenv_int("REPLAN_AUTO_COMMIT", 1) != 0);
    std::vector<int> keepNodes = newReserved;
    if (!autoCommit) {
        for (int nodeId : committedPrefix) {
            if (std::find(keepNodes.begin(), keepNodes.end(), nodeId) == keepNodes.end()) {
                keepNodes.push_back(nodeId);
            }
        }
    }
    reservations.releaseAllByOwnerExceptSet(deviceId, keepNodes);
    rt.reservedNodes = keepNodes;
    repo.updateRuntimeState(deviceId, planEntry.pathId, rt);
    if (autoCommit && !newReserved.empty()) {
        repo.updateLastSentRoute(deviceId, newReserved);
        if (logSummary) {
            std::cout << log_time_prefix()
                      << "[Replan] auto commit deviceId=" << deviceId
                      << " stage=" << replan_stage_label(stage)
                      << " committed=" << format_nodes_head(newReserved)
                      << " size=" << newReserved.size()
                      << std::endl;
        }
    }

    // Build response plan with a single segment (the reserved window).
    PathPlanningHelper::PathSegmentInfo seg;
    seg.fromNodeId = startNodeId;
    seg.toNodeId = newReserved.back();
    seg.nodes = newReserved;
    seg.reachable = true;
    seg.source = usedStatic ? GlobalPathPlanner::PathSource::STATIC_TABLE : GlobalPathPlanner::PathSource::PURE_ASTAR;
    if (target && !usingTempGoal) {
        seg.taskId = target->taskId;
        seg.subTaskId = target->subTaskId;
        seg.subTaskSequence = target->subTaskSequence;
        seg.stepType = (seg.toNodeId == primaryGoal) ? target->stepType : std::string("-1");
    } else {
        seg.taskId.clear();
        seg.subTaskId.clear();
        seg.subTaskSequence = -1;
        seg.stepType = "-1";
    }
    seg.distanceMm = compute_route_distance_mm(mapInfo, seg.nodes);
    double speed = mapInfo.getGlobalMaxSpeed();
    if (!(speed > 0.0)) speed = 1000.0;
    seg.timeSec = seg.distanceMm / speed;
    seg.serviceTimeSec = 0.0;
    seg.description = "dynamic_plan (" + std::to_string(seg.fromNodeId) + "->" + std::to_string(seg.toNodeId) + ")";

    out.segments.push_back(std::move(seg));
    out.totalDistanceMm = out.segments.front().distanceMm;
    out.totalTimeSec = out.segments.front().timeSec;
    out.allReachable = true;
    if (usedTempGoalOut) *usedTempGoalOut = usingTempGoal;
    return out;
}

static void trim_dynamic_plan_to_start(PathPlanningHelper::AmrPlanInfo& plan,
                                       int startNodeId,
                                       const MapInfo& mapInfo) {
    if (startNodeId < 0) return;
    if (plan.segments.empty()) {
        plan.startNodeId = startNodeId;
        return;
    }
    auto& seg = plan.segments.front();
    if (seg.nodes.empty()) {
        seg.nodes.push_back(startNodeId);
    } else {
        auto it = std::find(seg.nodes.begin(), seg.nodes.end(), startNodeId);
        if (it != seg.nodes.end()) {
            if (it != seg.nodes.begin()) {
                seg.nodes.erase(seg.nodes.begin(), it);
            }
        } else {
            seg.nodes.insert(seg.nodes.begin(), startNodeId);
        }
    }
    seg.fromNodeId = seg.nodes.front();
    seg.toNodeId = seg.nodes.back();
    plan.startNodeId = seg.fromNodeId;
    seg.distanceMm = compute_route_distance_mm(mapInfo, seg.nodes);
    double speed = mapInfo.getGlobalMaxSpeed();
    if (!(speed > 0.0)) speed = 1000.0;
    seg.timeSec = seg.distanceMm / speed;
    plan.totalDistanceMm = seg.distanceMm;
    plan.totalTimeSec = seg.timeSec;
}

static PathPlanningHelper::AmrPlanInfo compute_dynamic_plan_with_throttle(
    const std::string& deviceId,
    int startNodeId,
    int committedNextNodeId,
    int batteryLevel,
    int deviceType,
    const PlanCacheEntry& planEntry,
    RobotDataRepository& repo,
    const MapInfo& mapInfo,
    bool skipStaticTable,
    bool staticTableReady,
    StaticPathTable& staticTable,
    AStarPathFinder& aStar,
    NodeReservationTable& reservations,
    int reserveBudgetOverride,
    int minIntervalMs,
    bool* usedCacheOut) {
    if (usedCacheOut) *usedCacheOut = false;
    int baseIntervalMs = (minIntervalMs > 0) ? minIntervalMs : 200;
    int effectiveIntervalMs = baseIntervalMs;

    ReplanStage stage = ReplanStage::FAST;
    const int majorStuckMs = getenv_int("REPLAN_MAJOR_STUCK_MS", 2000);
    const int superAfterMajorFails = std::max(1, getenv_int("REPLAN_SUPER_AFTER_MAJOR_FAILS", 3));
    if (majorStuckMs >= 0) {
        PlanRuntimeState rt = repo.snapshotRuntimeState(deviceId);
        const auto now = std::chrono::steady_clock::now();
        long long staticMs = 0;
        if (rt.staticSince != std::chrono::steady_clock::time_point{}) {
            staticMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - rt.staticSince).count();
        }
        const bool staticLong = (staticMs >= majorStuckMs);
        const bool majorCooldownOk =
            (rt.lastMajorAttempt == std::chrono::steady_clock::time_point{}) ||
            (now - rt.lastMajorAttempt >= std::chrono::milliseconds(std::max(0, majorStuckMs)));
        if (staticLong && majorCooldownOk) {
            stage = (rt.majorFailCount >= superAfterMajorFails) ? ReplanStage::SUPER : ReplanStage::MAJOR;
        }
    }
    const bool allowLongerRoute = (stage == ReplanStage::MAJOR || stage == ReplanStage::SUPER);
    bool planUpdated = true;
    bool usedTempGoal = false;
    auto planned = compute_dynamic_plan_to_next_target(
        deviceId,
        startNodeId,
        committedNextNodeId,
        batteryLevel,
        deviceType,
        planEntry,
        repo,
        mapInfo,
        skipStaticTable,
        staticTableReady,
        staticTable,
        aStar,
        reservations,
        reserveBudgetOverride,
        allowLongerRoute,
        stage,
        &planUpdated,
        &usedTempGoal);
    if (effectiveIntervalMs > 0) {
        if (planUpdated) {
            bool refreshClock = (stage != ReplanStage::FAST) || usedTempGoal;
            repo.updateDynamicPlan(deviceId, planned, planEntry.pathId, reserveBudgetOverride, refreshClock);
        }
    } else if (!planUpdated && usedCacheOut) {
        *usedCacheOut = true;
    }
    return planned;
}

static std::string normalize_request_timestamp(const std::string& raw) {
    if (raw.empty()) return raw;
    bool digitOnly = std::all_of(raw.begin(), raw.end(), [](unsigned char c){ return std::isdigit(c); });
    if (digitOnly && raw.size() >= 10) {
        try {
            long long millis = std::stoll(raw);
            return iso8601_from_unix_millis(millis);
        } catch (...) {
            return raw;
        }
    }
    return raw;
}

static int read_update_time_sec(const json& item) {
    auto it = item.find("updateTime");
    if (it == item.end() || it->is_null()) return 0;
    if (it->is_number_integer()) {
        return StatusArbitration::NormalizeEpochSeconds(it->get<long long>());
    }
    if (it->is_number_unsigned()) {
        return StatusArbitration::NormalizeEpochSeconds(static_cast<long long>(it->get<unsigned long long>()));
    }
    if (it->is_number_float()) {
        return StatusArbitration::NormalizeEpochSeconds(static_cast<long long>(it->get<double>()));
    }
    if (it->is_string()) {
        return StatusArbitration::ParseUpdateTimeSec(it->get<std::string>());
    }
    return 0;
}

static bool fill_status_entry_from_json(const json& item, RobotStatusEntry& entry) {
    entry.deviceId = read_string(item, "deviceId", "");
    if (entry.deviceId.empty()) return false;
    entry.mapId = read_int(item, "mapId", 0);
    entry.curArea = read_string(item, "curArea", "");
    entry.connection = read_bool(item, "connection", true);
    entry.x = read_double(item, "x", 0.0);
    entry.y = read_double(item, "y", 0.0);
    entry.angle = read_double(item, "angle", 0.0);
    entry.speed = read_double(item, "speed", 0.0);
    entry.nodeId = read_string(item, "nodeId", "");
    if (item.contains("nextDestinationPoint")) entry.nextDestination = item["nextDestinationPoint"];
    entry.trailPoints.clear();
    if (item.contains("curTrailPoints") && item["curTrailPoints"].is_array()) {
        for (const auto& tp : item["curTrailPoints"]) entry.trailPoints.push_back(tp);
    }
    entry.taskId = read_string(item, "taskId", "");
    entry.taskStatus = read_int(item, "taskStatus", 0);
    entry.taskProgress = read_int(item, "taskProgress", 0);
    entry.batteryLevel = read_int(item, "batteryLevel", 0);
    entry.endurance = read_int(item, "endurance", 0);
    entry.load = read_bool(item, "load", false);
    entry.errorCode = read_int(item, "errorCode", 0);
    entry.updateTime = read_update_time_sec(item);
    if (item.contains("agv_type")) entry.deviceType = read_int(item, "agv_type", 0);
    else entry.deviceType = read_int(item, "deviceType", 0);
    entry.estimatedDurationSec = read_double(item, "estimatedDuration", 0.0);
    return true;
}

static size_t cache_agv_status_list(const json& j, RobotDataRepository& repo) {
    if (!j.contains("agvStatusList") || !j["agvStatusList"].is_array()) return 0;
    size_t count = 0;
    for (const auto& a : j["agvStatusList"]) {
        RobotStatusEntry entry;
        if (!fill_status_entry_from_json(a, entry)) continue;
        repo.updateStatus(entry, StatusSource::TASK_SNAPSHOT);
        ++count;
    }
    if (count && log_enabled(LogLevel::DEBUG)) {
        std::cout << log_time_prefix() << "[Status] cached " << count << " agvStatusList entries" << std::endl;
    }
    return count;
}

static Amr build_amr_from_status(const RobotStatusEntry& s,
                                 int resolvedAheadMs,
                                 int resolvedDetourMm,
                                 const MapInfo& /*mapInfo*/) {
    int currentNodeId = parse_node_id_str(s.nodeId);
    Point nxt;
    if (s.nextDestination.is_object()) {
        const auto& np = s.nextDestination;
        int nx = np.value("x", 0);
        int ny = np.value("y", 0);
        int na = np.value("angle", 0);
        int nid = -1;
        if (np.contains("nodeId")) {
            const auto& v = np["nodeId"];
            if (v.is_number_integer()) nid = v.get<int>();
            else if (v.is_string()) nid = parse_node_id_str(v.get<std::string>());
        }
        nxt = Point(nx, ny, na, nid, 0);
    } else {
        nxt = Point(0, 0, 0, -1, 0);
    }
    std::vector<Point> trail;
    for (const auto& p : s.trailPoints) {
        int px = p.value("x", 0);
        int py = p.value("y", 0);
        int pa = p.value("angle", 0);
        int pn = -1;
        if (p.contains("nodeId")) {
            const auto& v = p["nodeId"];
            if (v.is_number_integer()) pn = v.get<int>();
            else if (v.is_string()) pn = parse_node_id_str(v.get<std::string>());
        }
        trail.emplace_back(px, py, pa, pn, 0);
    }

    Amr amr(1, s.taskId, s.taskProgress, s.taskStatus, nxt, trail, s.curArea,
            s.x, s.y, s.angle, s.deviceId, s.deviceType, s.mapId);
    amr.setConnection(s.connection);
    amr.setSpeed(static_cast<int>(std::lround(s.speed)));
    amr.setCurrentNodeId(currentNodeId);
    amr.setEstimatedDurationSec(s.estimatedDurationSec);
    amr.setBatteryLevel(s.batteryLevel > 0 ? s.batteryLevel : 100);
    amr.setEndurance(s.endurance);
    amr.setLoad(s.load);
    amr.setErrorCode(s.errorCode);
    amr.setUpdateTime(s.updateTime);
    amr.setMaxTimeAheadMs(resolvedAheadMs);
    amr.setMaxDetourDistanceMm(resolvedDetourMm);
    return amr;
}

static std::vector<int> build_route_from_plan(const PathPlanningHelper::AmrPlanInfo& plan) {
    std::vector<int> nodes;
    for (const auto& seg : plan.segments) {
        if (seg.nodes.empty()) continue;
        if (nodes.empty()) {
            nodes.insert(nodes.end(), seg.nodes.begin(), seg.nodes.end());
        } else {
            if (!nodes.empty() && nodes.back() == seg.nodes.front()) {
                nodes.insert(nodes.end(), seg.nodes.begin() + 1, seg.nodes.end());
            } else {
                nodes.insert(nodes.end(), seg.nodes.begin(), seg.nodes.end());
            }
        }
    }
    return nodes;
}

struct SimPose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double velocity = 0.0;
    int nodeId = -1;
    bool valid = false;
};

static double compute_yaw_between_nodes(int fromNodeId, int toNodeId, const MapInfo& mapInfo) {
    try {
        const Node& from = mapInfo.getNodeById(fromNodeId);
        const Node& to = mapInfo.getNodeById(toNodeId);
        return std::atan2(to.y - from.y, to.x - from.x);
    } catch (...) {
        return 0.0;
    }
}

static bool fetch_node_position(int nodeId, const MapInfo& mapInfo, double& x, double& y) {
    try {
        const Node& node = mapInfo.getNodeById(nodeId);
        x = node.x;
        y = node.y;
        return true;
    } catch (...) {
        return false;
    }
}

static bool compute_pose_from_plan(const PathPlanningHelper::AmrPlanInfo& plan,
                                   const MapInfo& mapInfo,
                                   SimPose& pose) {
    auto route = build_route_from_plan(plan);
    int nodeId = plan.startNodeId;
    int nextNodeId = -1;
    if (!route.empty()) {
        nodeId = route.front();
        if (route.size() >= 2) nextNodeId = route[1];
    }
    double x = 0.0, y = 0.0;
    if (nodeId < 0 || !fetch_node_position(nodeId, mapInfo, x, y)) return false;
    pose.x = x;
    pose.y = y;
    pose.nodeId = nodeId;
    pose.yaw = (nextNodeId >= 0) ? compute_yaw_between_nodes(nodeId, nextNodeId, mapInfo) : 0.0;
    pose.velocity = 0.0;
    pose.valid = true;
    return true;
}

class SimPoseStreamer {
public:
    SimPoseStreamer(RobotDataRepository& repo,
                    MapInfo& mapInfo,
                    std::shared_mutex& mapMutex,
                    std::atomic<bool>& mapReady,
                    SimPosePublisher& publisher,
                    int intervalMs)
        : repo_(repo),
          mapInfo_(mapInfo),
          mapMutex_(mapMutex),
          mapReady_(mapReady),
          publisher_(publisher),
          intervalMs_(std::max(50, intervalMs)) {}

    ~SimPoseStreamer() {
        stop();
    }

    void start() {
        if (running_.exchange(true)) return;
        thread_ = std::thread([this]() { run(); });
    }

    void stop() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

private:
    void run() {
        while (running_.load()) {
            if (!mapReady_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs_));
                continue;
            }
            auto plans = repo_.snapshotPlans();
            auto statuses = repo_.snapshotStatuses();
            std::unordered_set<std::string> covered;
            std::vector<nlohmann::json> payloads;
            long long ts = current_millis();
            {
                std::shared_lock<std::shared_mutex> lock(mapMutex_);
                for (auto& kv : plans) {
                    const std::string& deviceId = kv.first;
                    SimPose pose;
                    if (!compute_pose_from_plan(kv.second.plan, mapInfo_, pose)) continue;
                    nlohmann::json body;
                    body["deviceId"] = deviceId;
                    body["timestamp"] = ts;
                    body["x"] = pose.x;
                    body["y"] = pose.y;
                    body["yaw"] = pose.yaw;
                    body["velocity"] = pose.velocity;
                    body["nodeId"] = node_id_to_string(pose.nodeId);
                    body["source"] = "plan";
                    body["schedMessageId"] = kv.second.schedulingRequestId;
                    payloads.push_back(std::move(body));
                    covered.insert(deviceId);
                }
            }
            auto deg2rad = [](double deg) {
                return deg * M_PI / 180.0;
            };
            for (const auto& st : statuses) {
                if (covered.count(st.deviceId)) continue;
                nlohmann::json body;
                body["deviceId"] = st.deviceId;
                body["timestamp"] = ts;
                body["x"] = st.x;
                body["y"] = st.y;
                body["yaw"] = deg2rad(st.angle);
                body["velocity"] = st.speed;
                body["nodeId"] = st.nodeId;
                body["source"] = "status";
                payloads.push_back(std::move(body));
            }
            for (const auto& body : payloads) {
                publisher_.publishPose(body);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs_));
        }
    }

    RobotDataRepository& repo_;
    MapInfo& mapInfo_;
    std::shared_mutex& mapMutex_;
    std::atomic<bool>& mapReady_;
    SimPosePublisher& publisher_;
    int intervalMs_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

class SimReservedStreamer {
public:
    SimReservedStreamer(NodeReservationTable& reservations,
                        std::atomic<bool>& mapReady,
                        SimReservedPublisher& publisher,
                        int intervalMs)
        : reservations_(reservations),
          mapReady_(mapReady),
          publisher_(publisher),
          intervalMs_(std::max(100, intervalMs)) {
        dumpPath_ = getenv_str("SIM_RESERVED_DUMP_PATH", "");
        dumpIntervalMs_ = std::max(0, getenv_int("SIM_RESERVED_DUMP_INTERVAL_MS", intervalMs_));
        if (dumpPath_.empty()) dumpIntervalMs_ = 0;
    }

    ~SimReservedStreamer() {
        stop();
    }

    void start() {
        if (running_.exchange(true)) return;
        thread_ = std::thread([this]() { run(); });
    }

    void stop() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

private:
    void run() {
        while (running_.load()) {
            if (!mapReady_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs_));
                continue;
            }
            maybe_cleanup_reservations(reservations_);
            auto heldNodes = reservations_.snapshotHeldNodes();
            nlohmann::json body;
            body["timestamp"] = current_millis();
            nlohmann::json nodes = nlohmann::json::array();
            for (int nodeId : heldNodes) {
                nlohmann::json item;
                item["nodeId"] = nodeId;
                auto hold = reservations_.get(nodeId);
                if (hold.has_value()) {
                    item["owner"] = hold->ownerAgvId;
                    item["reason"] = static_cast<int>(hold->reason);
                    if (!hold->detail.empty()) {
                        item["detail"] = hold->detail;
                    }
                }
                nodes.push_back(std::move(item));
            }
            body["reservedNodes"] = std::move(nodes);
            publisher_.publishSnapshot(body);
            dump_snapshot_if_needed(body);
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs_));
        }
    }

    void dump_snapshot_if_needed(const json& body) {
        if (dumpIntervalMs_ <= 0 || dumpPath_.empty()) return;
        auto now = std::chrono::steady_clock::now();
        if (lastDump_ != std::chrono::steady_clock::time_point{} &&
            (now - lastDump_) < std::chrono::milliseconds(dumpIntervalMs_)) {
            return;
        }
        lastDump_ = now;
        std::error_code ec;
        std::filesystem::path outPath(dumpPath_);
        if (outPath.has_parent_path()) {
            std::filesystem::create_directories(outPath.parent_path(), ec);
        }
        std::ofstream out(dumpPath_, std::ios::trunc);
        if (!out) return;
        out << body.dump(2);
    }

    NodeReservationTable& reservations_;
    std::atomic<bool>& mapReady_;
    SimReservedPublisher& publisher_;
    int intervalMs_;
    std::string dumpPath_;
    int dumpIntervalMs_{0};
    std::chrono::steady_clock::time_point lastDump_{};
    std::atomic<bool> running_{false};
    std::thread thread_;
};

class MultiMapSimPoseStreamer {
public:
    MultiMapSimPoseStreamer(MultiMapManager& mapManager,
                           SimPosePublisher& publisher,
                           int intervalMs)
        : mapManager_(mapManager),
          publisher_(publisher),
          intervalMs_(std::max(50, intervalMs)) {}

    ~MultiMapSimPoseStreamer() {
        stop();
    }

    void start() {
        if (running_.exchange(true)) return;
        thread_ = std::thread([this]() { run(); });
    }

    void stop() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

private:
    void run() {
        while (running_.load()) {
            auto contexts = mapManager_.snapshotContexts();
            long long ts = current_millis();
            for (const auto& ctx : contexts) {
                if (!ctx->mapReady.load()) continue;
                auto plans = ctx->robotRepo.snapshotPlans();
                auto statuses = ctx->robotRepo.snapshotStatuses();
                std::unordered_set<std::string> covered;
                std::vector<nlohmann::json> payloads;
                {
                    std::shared_lock<std::shared_mutex> lock(ctx->mapMutex);
                    for (auto& kv : plans) {
                        const std::string& deviceId = kv.first;
                        SimPose pose;
                        if (!compute_pose_from_plan(kv.second.plan, *ctx->mapInfoPtr, pose)) continue;
                        nlohmann::json body;
                        body["mapId"] = ctx->activeMapId.load();
                        if (!ctx->mapVersion.empty()) body["mapVersion"] = ctx->mapVersion;
                        body["deviceId"] = deviceId;
                        body["timestamp"] = ts;
                        body["x"] = pose.x;
                        body["y"] = pose.y;
                        body["yaw"] = pose.yaw;
                        body["velocity"] = pose.velocity;
                        body["nodeId"] = node_id_to_string(pose.nodeId);
                        body["source"] = "plan";
                        body["schedMessageId"] = kv.second.schedulingRequestId;
                        payloads.push_back(std::move(body));
                        covered.insert(deviceId);
                    }
                }
                auto deg2rad = [](double deg) {
                    return deg * M_PI / 180.0;
                };
                for (const auto& st : statuses) {
                    if (covered.count(st.deviceId)) continue;
                    nlohmann::json body;
                    body["mapId"] = ctx->activeMapId.load();
                    if (!ctx->mapVersion.empty()) body["mapVersion"] = ctx->mapVersion;
                    body["deviceId"] = st.deviceId;
                    body["timestamp"] = ts;
                    body["x"] = st.x;
                    body["y"] = st.y;
                    body["yaw"] = deg2rad(st.angle);
                    body["velocity"] = st.speed;
                    body["nodeId"] = st.nodeId;
                    body["source"] = "status";
                    payloads.push_back(std::move(body));
                }
                for (const auto& body : payloads) {
                    publisher_.publishPose(body);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs_));
        }
    }

    MultiMapManager& mapManager_;
    SimPosePublisher& publisher_;
    int intervalMs_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

class MultiMapSimReservedStreamer {
public:
    MultiMapSimReservedStreamer(MultiMapManager& mapManager,
                               SimReservedPublisher& publisher,
                               int intervalMs)
        : mapManager_(mapManager),
          publisher_(publisher),
          intervalMs_(std::max(100, intervalMs)) {
        dumpPath_ = getenv_str("SIM_RESERVED_DUMP_PATH", "");
        dumpIntervalMs_ = std::max(0, getenv_int("SIM_RESERVED_DUMP_INTERVAL_MS", intervalMs_));
        if (dumpPath_.empty()) dumpIntervalMs_ = 0;
    }

    ~MultiMapSimReservedStreamer() {
        stop();
    }

    void start() {
        if (running_.exchange(true)) return;
        thread_ = std::thread([this]() { run(); });
    }

    void stop() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

private:
    void run() {
        while (running_.load()) {
            auto contexts = mapManager_.snapshotContexts();
            nlohmann::json body;
            body["timestamp"] = current_millis();
            nlohmann::json nodes = nlohmann::json::array();
            for (const auto& ctx : contexts) {
                if (!ctx->mapReady.load()) continue;
                maybe_cleanup_reservations(ctx->nodeReservations);
                auto heldNodes = ctx->nodeReservations.snapshotHeldNodes();
                for (int nodeId : heldNodes) {
                    nlohmann::json item;
                    item["mapId"] = ctx->activeMapId.load();
                    if (!ctx->mapVersion.empty()) item["mapVersion"] = ctx->mapVersion;
                    item["nodeId"] = nodeId;
                    auto hold = ctx->nodeReservations.get(nodeId);
                    if (hold.has_value()) {
                        item["owner"] = hold->ownerAgvId;
                        item["reason"] = static_cast<int>(hold->reason);
                        if (!hold->detail.empty()) {
                            item["detail"] = hold->detail;
                        }
                    }
                    nodes.push_back(std::move(item));
                }
            }
            body["reservedNodes"] = std::move(nodes);
            publisher_.publishSnapshot(body);
            dump_snapshot_if_needed(body);
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs_));
        }
    }

    void dump_snapshot_if_needed(const json& body) {
        if (dumpIntervalMs_ <= 0 || dumpPath_.empty()) return;
        auto now = std::chrono::steady_clock::now();
        if (lastDump_ != std::chrono::steady_clock::time_point{} &&
            (now - lastDump_) < std::chrono::milliseconds(dumpIntervalMs_)) {
            return;
        }
        lastDump_ = now;
        std::error_code ec;
        std::filesystem::path outPath(dumpPath_);
        if (outPath.has_parent_path()) {
            std::filesystem::create_directories(outPath.parent_path(), ec);
        }
        std::ofstream out(dumpPath_, std::ios::trunc);
        if (!out) return;
        out << body.dump(2);
    }

    MultiMapManager& mapManager_;
    SimReservedPublisher& publisher_;
    int intervalMs_;
    std::string dumpPath_;
    int dumpIntervalMs_{0};
    std::chrono::steady_clock::time_point lastDump_{};
    std::atomic<bool> running_{false};
    std::thread thread_;
};

static size_t count_assigned_tasks(const AllocationResult& result) {
    size_t count = 0;
    for (const auto& seq : result.amrTasks) {
        count += seq.size();
    }
    return count;
}

static void collect_vehicle_filters(const json& taskObj, std::vector<std::string>& out) {
    out.clear();
    if (!taskObj.contains("agvRequirements")) return;
    const auto& req = taskObj["agvRequirements"];
    auto push_val = [&](const json& val) {
        if (val.is_string()) {
            out.push_back(val.get<std::string>());
        } else if (val.is_number_integer()) {
            out.push_back("AGV" + std::to_string(val.get<int>()));
        }
    };
    if (req.is_array()) {
        for (const auto& v : req) {
            push_val(v);
        }
        return;
    }
    if (req.is_string() || req.is_number_integer()) {
        push_val(req);
        return;
    }
    if (req.is_object()) {
        // 兼容旧格式：agvRequirements.vehicleTypes/vehicleIds
        if (req.contains("vehicleTypes") && req["vehicleTypes"].is_array()) {
            for (const auto& vt : req["vehicleTypes"]) push_val(vt);
        }
        if (req.contains("vehicleIds") && req["vehicleIds"].is_array()) {
            for (const auto& vid : req["vehicleIds"]) push_val(vid);
        }
    }
}

static std::string parse_agv_id(const json& obj) {
    auto to_str = [](const json& val) -> std::string {
        if (val.is_string()) return val.get<std::string>();
        if (val.is_number_integer()) return std::to_string(val.get<int>());
        if (val.is_number_unsigned()) return std::to_string(val.get<unsigned int>());
        return std::string();
    };
    if (obj.contains("agv_id")) {
        auto id = to_str(obj["agv_id"]);
        if (!id.empty()) return id;
    }
    if (obj.contains("agvId")) {
        auto id = to_str(obj["agvId"]);
        if (!id.empty()) return id;
    }
    if (obj.contains("deviceId")) {
        auto id = to_str(obj["deviceId"]);
        if (!id.empty()) return id;
    }
    return std::string();
}

static void handle_status_infos(const json& j,
                                RobotDataRepository& repo,
                                bool mapReadyNow,
                                const MapInfo& mapInfo,
                                NodeReservationTable& reservations) {
    const json* statusArray = nullptr;
    if (j.is_array()) {
        statusArray = &j;
    } else if (j.contains("robotStatusInfos") && j["robotStatusInfos"].is_array()) {
        statusArray = &j["robotStatusInfos"];
    } else if (j.contains("RobotStatusInfos") && j["RobotStatusInfos"].is_array()) {
        statusArray = &j["RobotStatusInfos"];
    }
    if (!statusArray) {
        if (log_enabled(LogLevel::DEBUG)) {
            std::cout << log_time_prefix()
                      << "[Status] invalid payload, missing robotStatusInfos" << std::endl;
        }
        return;
    }
    const bool logSummary = env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL");
    const bool logStatusNodeXY = env_enabled("RECEIVER_LOG_STATUS_NODE_XY");
    maybe_cleanup_reservations(reservations);
    const int ttlMs = getenv_int("STATUS_HOLD_TTL_MS", getenv_int("RESERVE_TTL_MS", 15000));
    const int committedTtlMs = std::max(0, getenv_int("RESERVE_COMMITTED_TTL_MS", 0));
    size_t count = 0;
    for (const auto& item : *statusArray) {
        RobotStatusEntry entry;
        if (!fill_status_entry_from_json(item, entry)) continue;
        record_status_lag(entry);
        repo.updateStatus(entry, StatusSource::STATUS_STREAM);
        const bool idleNow = status_is_idle(entry);
        const bool idleReleaseEnabled = (getenv_int("STATUS_IDLE_RELEASE_ENABLE", 1) != 0);
        int curNodeId = -1;
        int rawNodeId = parse_node_id_str(entry.nodeId);
        PlanRuntimeState rt;
        bool rtReady = false;
        if (mapReadyNow) {
            curNodeId = resolve_status_node_id(entry, mapInfo);
            if (curNodeId >= 0) {
                rt = repo.snapshotRuntimeState(entry.deviceId);
                rtReady = true;
            }
        }
        if (logStatusNodeXY) {
            std::cout << log_time_prefix()
                      << "[StatusDebug] deviceId=" << entry.deviceId
                      << " nodeId=" << (entry.nodeId.empty() ? "<none>" : entry.nodeId)
                      << " parsed=" << rawNodeId
                      << " resolved=" << curNodeId
                      << " taskStatus=" << entry.taskStatus
                      << " taskId=" << (entry.taskId.empty() ? "<empty>" : entry.taskId)
                      << " x=" << static_cast<int>(std::lround(entry.x))
                      << " y=" << static_cast<int>(std::lround(entry.y))
                      << std::endl;
            std::cout << log_time_prefix()
                      << "[StatusFull] deviceId=" << entry.deviceId
                      << " payload=" << item.dump()
                      << std::endl;
        }
        if (mapReadyNow) {
            if (curNodeId >= 0) {
                bool skipRefreshHolds = false;
                // Update target progress for this AGV.
                repo.advanceTargets(entry.deviceId, curNodeId);

                // Release reserved window behind current node.
                if (!rtReady) {
                    rt = repo.snapshotRuntimeState(entry.deviceId);
                    rtReady = true;
                }
                const bool enableTaskStatusGate = (getenv_int("REPLAN_TASK_STATUS_GATE_ENABLE", 0) != 0);
                if (enableTaskStatusGate && rt.waitTaskStatusClear) {
                    reservations.releaseAllByOwner(entry.deviceId);
                    rt.reservedNodes.clear();
                    rt.blockedSince = std::chrono::steady_clock::time_point{};
                    rt.tempGoalNodeId = -1;
                    rt.tempGoalBans.clear();
                    rt.tempGoalTabu.clear();
                    rt.tempGoalFreezeUntil = std::chrono::steady_clock::time_point{};
                    rt.staticSince = std::chrono::steady_clock::time_point{};
                    rt.staticNodeId = -1;
                    rt.statusMismatchSince = std::chrono::steady_clock::time_point{};
                    rt.statusMismatchNodeId = -1;
                    repo.updateReservedNodesAndMismatchOnly(entry.deviceId,
                                                            std::vector<int>{},
                                                            rt.statusMismatchSince,
                                                            rt.statusMismatchNodeId);
                    repo.updateLastSentRoute(entry.deviceId, {});
                    repo.updateRuntimeState(entry.deviceId, rt.planPathId, rt);
                    if (logSummary) {
                        std::cout << log_time_prefix()
                                  << "[Reserve] gate hold deviceId=" << entry.deviceId
                                  << " taskStatus=" << entry.taskStatus
                                  << " node=" << curNodeId
                                  << std::endl;
                    }
                    skipRefreshHolds = true;
                } else {
                    bool idleReleaseNow = false;
                    if (idleReleaseEnabled) {
                        // If all target points are consumed, treat as idle to allow release.
                        const bool idleByTargets = !repo.peekTarget(entry.deviceId).has_value();
                        idleReleaseNow = idleNow || idleByTargets;
                    }
                    if (idleReleaseNow) {
                        std::vector<int> keepNodes;
                        keepNodes.push_back(curNodeId);
                        reservations.releaseAllByOwnerExceptSet(entry.deviceId, keepNodes);
                        reservations.tryReserve(curNodeId,
                                                 entry.deviceId,
                                                 NodeReservationTable::HoldReason::WAITING_POINT,
                                                 "idle",
                                                 std::chrono::milliseconds(0));
                        rt.reservedNodes = keepNodes;
                        rt.statusMismatchSince = std::chrono::steady_clock::time_point{};
                        rt.statusMismatchNodeId = -1;
                        repo.updateReservedNodesAndMismatchOnly(entry.deviceId,
                                                                keepNodes,
                                                                rt.statusMismatchSince,
                                                                rt.statusMismatchNodeId);
                        repo.updateLastSentRoute(entry.deviceId, keepNodes);
                        if (logSummary) {
                            std::cout << log_time_prefix()
                                      << "[Reserve] idle release deviceId=" << entry.deviceId
                                      << " keepNode=" << curNodeId
                                      << std::endl;
                        }
                        if (logSummary) {
                            std::cout << log_time_prefix()
                                      << "[Reserve] idle release status deviceId=" << entry.deviceId
                                      << " taskStatus=" << entry.taskStatus
                                      << " taskId=" << (entry.taskId.empty() ? "<empty>" : entry.taskId)
                                      << " nodeId=" << (entry.nodeId.empty() ? "<empty>" : entry.nodeId)
                                      << std::endl;
                        }
                        skipRefreshHolds = true;
                    } else if (!rt.reservedNodes.empty()) {
                    const auto now = std::chrono::steady_clock::now();
                    const long long mismatchResetMs = getenv_int("STATUS_MISMATCH_RESET_MS", 2000);
                    std::vector<int> trimmed = rt.reservedNodes;
                    bool changed = false;
                    bool runtimeDirty = false;
                    bool resetPrefix = false;
                    auto it = std::find(trimmed.begin(), trimmed.end(), curNodeId);
                    if (it != trimmed.end()) {
                        size_t releasedCount = static_cast<size_t>(std::distance(trimmed.begin(), it));
                        std::string oldHead = format_nodes_head(trimmed);
                        for (auto jt = trimmed.begin(); jt != it; ++jt) {
                            reservations.release(*jt, entry.deviceId);
                        }
                        trimmed.erase(trimmed.begin(), it);
                        changed = true;
                        if (logSummary && releasedCount > 0) {
                            std::cout << log_time_prefix()
                                      << "[Reserve] release deviceId=" << entry.deviceId
                                      << " curNode=" << curNodeId
                                      << " count=" << releasedCount
                                      << " oldHead=" << oldHead
                                      << " newHead=" << format_nodes_head(trimmed)
                                      << " newSize=" << trimmed.size()
                                      << std::endl;
                        }
                        if (rt.statusMismatchSince != std::chrono::steady_clock::time_point{} ||
                            rt.statusMismatchNodeId != -1) {
                            rt.statusMismatchSince = std::chrono::steady_clock::time_point{};
                            rt.statusMismatchNodeId = -1;
                            runtimeDirty = true;
                        }
                    } else {
                        // Status nodeId is not inside the reserved/committed prefix.
                        // Keep it for a short grace period (status might lag), then reset to avoid
                        // permanent "committed" blocks caused by prefix desync.
                        if (mismatchResetMs >= 0) {
                            if (rt.statusMismatchSince == std::chrono::steady_clock::time_point{}) {
                                rt.statusMismatchSince = now;
                                rt.statusMismatchNodeId = curNodeId;
                                runtimeDirty = true;
                            } else if (rt.statusMismatchNodeId != curNodeId) {
                                // Keep the original mismatch start time to avoid
                                // perpetually resetting when the status keeps moving.
                                rt.statusMismatchNodeId = curNodeId;
                                runtimeDirty = true;
                            }
                            if (mismatchResetMs == 0 ||
                                (now - rt.statusMismatchSince) >= std::chrono::milliseconds(mismatchResetMs)) {
                                resetPrefix = true;
                            }
                        }
                        if (resetPrefix) {
                            if (env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL")) {
                                std::cout << log_time_prefix()
                                          << "[Reserve] reset prefix on status mismatch deviceId=" << entry.deviceId
                                          << " curNode=" << curNodeId
                                          << " reservedHead=" << format_nodes_head(trimmed)
                                          << " reservedSize=" << trimmed.size()
                                          << std::endl;
                            }
                            trimmed.clear();
                            trimmed.push_back(curNodeId);
                            reservations.releaseAllByOwnerExceptSet(entry.deviceId, trimmed);
                            changed = true;
                            rt.statusMismatchSince = std::chrono::steady_clock::time_point{};
                            rt.statusMismatchNodeId = -1;
                            runtimeDirty = true;
                        } else if (env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL")) {
                            std::cout << log_time_prefix()
                                      << "[Reserve] keep prefix on status mismatch deviceId=" << entry.deviceId
                                      << " curNode=" << curNodeId
                                      << " reservedHead=" << format_nodes_head(trimmed)
                                      << " reservedSize=" << trimmed.size()
                                      << std::endl;
                        }
                    }
                    if (changed) runtimeDirty = true;
                    if (runtimeDirty) {
                        repo.updateReservedNodesAndMismatchOnly(entry.deviceId,
                                                                trimmed,
                                                                rt.statusMismatchSince,
                                                                rt.statusMismatchNodeId);
                    }
                    std::vector<int> lastSent = repo.getLastSentRoute(entry.deviceId);
                    if (!lastSent.empty()) {
                        bool adjusted = false;
                        if (resetPrefix) {
                            lastSent.clear();
                            lastSent.push_back(curNodeId);
                            adjusted = true;
                        } else {
                            auto itTrail = std::find(lastSent.begin(), lastSent.end(), curNodeId);
                            if (itTrail != lastSent.end()) {
                                if (itTrail != lastSent.begin()) {
                                    lastSent.erase(lastSent.begin(), itTrail);
                                    adjusted = true;
                                }
                            } else {
                                if (env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL")) {
                                    std::cout << log_time_prefix()
                                              << "[TrailResp] keep committed prefix on status mismatch deviceId=" << entry.deviceId
                                              << " curNode=" << curNodeId
                                              << " lastSentHead=" << format_nodes_head(lastSent)
                                              << " lastSentSize=" << lastSent.size()
                                              << std::endl;
                                }
                            }
                        }
                        if (adjusted) {
                            repo.updateLastSentRoute(entry.deviceId, lastSent);
                        }
                    } else if (resetPrefix) {
                        // lastSentRoute empty but we decided to reset prefix; keep it consistent.
                        repo.updateLastSentRoute(entry.deviceId, std::vector<int>{curNodeId});
                    }
                }
                }

                if (!skipRefreshHolds) {
                    // Refresh holds for current+reserved window (best-effort).
                    std::vector<int> committed = repo.getLastSentRoute(entry.deviceId);
                    std::unordered_set<int> committedSet(committed.begin(), committed.end());
                    std::vector<int> reservedNow = repo.snapshotRuntimeState(entry.deviceId).reservedNodes;
                    bool refreshedCur = false;
                    auto refresh_hold = [&](int nodeId,
                                            NodeReservationTable::HoldReason reason,
                                            const std::string& detail) {
                        if (nodeId < 0) return;
                        int ttlUse = committedSet.count(nodeId) > 0 ? committedTtlMs : ttlMs;
                        if (ttlUse < 0) ttlUse = 0;
                        if (reason == NodeReservationTable::HoldReason::WAITING_POINT) {
                            ttlUse = 0;
                        }
                        if (!reservations.tryReserve(nodeId,
                                                     entry.deviceId,
                                                     reason,
                                                     detail,
                                                     std::chrono::milliseconds(std::max(0, ttlUse)))) {
                            auto hold = reservations.findBlockingHold(nodeId, entry.deviceId);
                            if (hold.has_value() && hold->ownerAgvId != entry.deviceId) {
                                std::cout << log_time_prefix()
                                          << "[Reserve] conflict deviceId=" << entry.deviceId
                                          << " node=" << nodeId
                                          << " holder=" << hold->ownerAgvId
                                          << " reason=" << hold_reason_label(hold->reason)
                                          << " detail=" << (hold->detail.empty() ? "<none>" : hold->detail)
                                          << std::endl;
                            }
                        }
                    };
                    for (int nodeId : reservedNow) {
                        if (nodeId == curNodeId) refreshedCur = true;
                        NodeReservationTable::HoldReason reason = nodeId == curNodeId
                            ? NodeReservationTable::HoldReason::WAITING_POINT
                            : NodeReservationTable::HoldReason::RESERVED_PATH;
                        refresh_hold(nodeId, reason, nodeId == curNodeId ? "status" : "status_refresh");
                    }
                    if (!refreshedCur) {
                        refresh_hold(curNodeId, NodeReservationTable::HoldReason::WAITING_POINT, "status");
                    }
                    // External status stream may not provide nextDestinationPoint; avoid relying on it.
                    // Enable explicit next-node holds only when the upstream guarantees correctness.
                    if (env_enabled("RESERVE_STATUS_NEXT")) {
                        int nextNodeId = resolve_committed_next_node_id(entry, mapInfo);
                        if (nextNodeId >= 0 && nextNodeId != curNodeId) {
                            refresh_hold(nextNodeId,
                                         NodeReservationTable::HoldReason::RESERVED_PATH,
                                         "status_next");
                        }
                    }
                }
            }
        }
        ++count;
    }
    maybe_release_stale_reservations(repo, reservations);
    maybe_log_status_update(count);
}

static void handle_config_infos(const json& j, RobotDataRepository& repo) {
    const json* configArray = nullptr;
    if (j.is_array()) {
        configArray = &j;
    } else if (j.contains("robotConfigInfos") && j["robotConfigInfos"].is_array()) {
        configArray = &j["robotConfigInfos"];
    } else if (j.contains("RobotConfigInfos") && j["RobotConfigInfos"].is_array()) {
        configArray = &j["RobotConfigInfos"];
    }
    if (!configArray) {
        std::cerr << "[Config] invalid payload, missing robotConfigInfos" << std::endl;
        return;
    }
    size_t count = 0;
    for (const auto& item : *configArray) {
        RobotConfigEntry entry;
        entry.agvId = parse_agv_id(item);
        if (entry.agvId.empty()) continue;
        entry.rawConfig = item;
        repo.updateConfig(entry);
        ++count;
    }
    if (log_enabled(LogLevel::DEBUG) && count > 0) {
        std::cout << log_time_prefix() << "[Config] updated " << count << " robot config entries" << std::endl;
    }
}

static void handle_status_infos_multimap(const json& j,
                                         MultiMapManager& mapManager) {
    const json* statusArray = nullptr;
    if (j.is_array()) {
        statusArray = &j;
    } else if (j.contains("robotStatusInfos") && j["robotStatusInfos"].is_array()) {
        statusArray = &j["robotStatusInfos"];
    } else if (j.contains("RobotStatusInfos") && j["RobotStatusInfos"].is_array()) {
        statusArray = &j["RobotStatusInfos"];
    }
    if (!statusArray) {
        if (log_enabled(LogLevel::DEBUG)) {
            std::cout << log_time_prefix()
                      << "[Status] invalid payload, missing robotStatusInfos" << std::endl;
        }
        return;
    }

    std::unordered_map<int, json> grouped;
    for (const auto& item : *statusArray) {
        RobotStatusEntry entry;
        if (!fill_status_entry_from_json(item, entry)) continue;
        int mapId = infer_status_map_id(entry, mapManager);
        if (mapId <= 0) {
            auto onlyCtx = mapManager.singleReadyContext();
            if (onlyCtx) mapId = onlyCtx->activeMapId.load();
        }
        if (mapId <= 0) {
            if (log_enabled(LogLevel::DEBUG)) {
                std::cout << log_time_prefix() << "[Status] skip device without mapId deviceId="
                          << entry.deviceId << std::endl;
            }
            continue;
        }
        mapManager.moveDeviceToContext(entry.deviceId, mapId);
        auto ctx = mapManager.getOrCreateContext(mapId);
        auto cfgOpt = mapManager.getConfig(entry.deviceId);
        if (cfgOpt.has_value()) {
            ctx->robotRepo.updateConfig(*cfgOpt);
        }
        json normalized = item;
        normalized["mapId"] = mapId;
        auto& arr = grouped[mapId];
        if (!arr.is_array()) arr = json::array();
        arr.push_back(std::move(normalized));
    }

    for (auto& kv : grouped) {
        auto ctx = mapManager.getOrCreateContext(kv.first);
        std::shared_lock<std::shared_mutex> mapReadLock(ctx->mapMutex);
        const MapInfo& mapInfo = *ctx->mapInfoPtr;
        handle_status_infos(kv.second,
                            ctx->robotRepo,
                            ctx->mapReady.load(),
                            mapInfo,
                            ctx->nodeReservations);
    }
}

static void handle_config_infos_multimap(const json& j,
                                         MultiMapManager& mapManager) {
    const json* configArray = nullptr;
    if (j.is_array()) {
        configArray = &j;
    } else if (j.contains("robotConfigInfos") && j["robotConfigInfos"].is_array()) {
        configArray = &j["robotConfigInfos"];
    } else if (j.contains("RobotConfigInfos") && j["RobotConfigInfos"].is_array()) {
        configArray = &j["RobotConfigInfos"];
    }
    if (!configArray) {
        std::cerr << "[Config] invalid payload, missing robotConfigInfos" << std::endl;
        return;
    }
    size_t count = 0;
    for (const auto& item : *configArray) {
        RobotConfigEntry entry;
        entry.agvId = parse_agv_id(item);
        if (entry.agvId.empty()) continue;
        entry.rawConfig = item;
        mapManager.updateConfig(entry);
        auto mapped = mapManager.getDeviceMap(entry.agvId);
        if (mapped.has_value()) {
            mapManager.getOrCreateContext(*mapped)->robotRepo.updateConfig(entry);
        } else if (auto onlyCtx = mapManager.singleReadyContext()) {
            onlyCtx->robotRepo.updateConfig(entry);
        }
        ++count;
    }
    if (log_enabled(LogLevel::DEBUG) && count > 0) {
        std::cout << log_time_prefix() << "[Config] updated " << count << " robot config entries" << std::endl;
    }
}

static double reserve_near_conflict_threshold_mm() {
    double thresholdMm = getenv_double("RESERVE_NEAR_CONFLICT_MM", -1.0);
    if (std::isfinite(thresholdMm) && thresholdMm >= 0.0) {
        return thresholdMm;
    }
    double thresholdM = getenv_double("RESERVE_NEAR_CONFLICT_M", 0.49);
    if (!std::isfinite(thresholdM) || thresholdM < 0.0) {
        thresholdM = 0.49;
    }
    return thresholdM * 1000.0;
}

// 输出地图中最大 ID 的节点信息，便于核对接收到的属性
static bool handle_map_info(const json& j,
                            MapInfo& mapInfo,
                            NodeReservationTable& reservations,
                            std::unique_ptr<AStarPathFinder>& aStarPtr,
                            const std::string& cachePath,
                            std::atomic<int>& mapId) {
    auto read_int_field = [](const json& obj, const char* key, int def) -> int {
        auto it = obj.find(key);
        if (it == obj.end() || it->is_null()) return def;
        if (it->is_number_integer()) return it->get<int>();
        if (it->is_number_unsigned()) return static_cast<int>(it->get<unsigned int>());
        if (it->is_string()) {
            try { return std::stoi(it->get<std::string>()); } catch (...) {}
        }
        return def;
    };
    const json* root = &j;
    json arrayHolder;
    if (j.is_array()) {
        if (j.empty()) {
            std::cerr << "[Map] payload array empty" << std::endl;
            return false;
        }
        arrayHolder = j.front();
        root = &arrayHolder;
    }

    const json* mapBlob = nullptr;
    json parsedBlob;
    if (root->contains("mapData")) {
        const auto& raw = root->at("mapData");
        if (raw.is_object()) {
            mapBlob = &raw;
        } else if (raw.is_string()) {
            try {
                parsedBlob = json::parse(raw.get<std::string>());
                mapBlob = &parsedBlob;
            } catch (const std::exception& ex) {
                std::cerr << "[Map] mapData string parse failed: " << ex.what() << std::endl;
                return false;
            }
        }
    } else if (root->is_object()) {
        if (root->contains("nodes") && root->contains("edges")) {
            mapBlob = root;
        } else if (root->contains("node") && root->contains("edge")) {
            mapBlob = root;
        }
    }
    if (!mapBlob) {
        std::cerr << "[Map] payload missing mapData" << std::endl;
        return false;
    }
    bool hasMapId = false;
    int parsedMapId = 0;
    if (root->is_object() && root->contains("mapId")) {
        parsedMapId = read_int_field(*root, "mapId", 0);
        hasMapId = true;
    }
    if (!hasMapId && mapBlob->is_object()) {
        if (mapBlob->contains("info") && (*mapBlob)["info"].is_object()
            && (*mapBlob)["info"].contains("mapId")) {
            parsedMapId = read_int_field((*mapBlob)["info"], "mapId", 0);
            hasMapId = true;
        } else if (mapBlob->contains("mapId")) {
            parsedMapId = read_int_field(*mapBlob, "mapId", 0);
            hasMapId = true;
        }
    }
    const bool startupSummary = env_enabled("RECEIVER_LOG_STARTUP_SUMMARY");
    try {
        JsonParser parser;
        std::vector<Node> nodes;
        std::vector<Edge> edges;
        std::vector<Area> areas;
        double maxv = 0.0;
        parser.parseMapDocument(*mapBlob, nodes, edges, areas, maxv);
        mapInfo.loadNodes(nodes, areas, maxv);
        mapInfo.loadEdges(edges);
        const double nearConflictThresholdMm = std::max(0.0, reserve_near_conflict_threshold_mm());
        reservations.configureDistanceConflict(mapInfo, nearConflictThresholdMm);
        aStarPtr.reset(new AStarPathFinder(mapInfo));
        if (!cachePath.empty()) {
            try {
                std::filesystem::path cacheDir = std::filesystem::path(cachePath).parent_path();
                if (!cacheDir.empty()) {
                    std::filesystem::create_directories(cacheDir);
                }
                std::ofstream ofs(cachePath, std::ios::out | std::ios::binary);
                if (ofs) {
                    ofs << mapBlob->dump(2);
                    ofs.close();
                    if (startupSummary) {
                        std::cout << "[Map] cached map JSON to " << cachePath << std::endl;
                    }
                }
            } catch (const std::exception& ex) {
                std::cerr << "[Map] failed to cache map json: " << ex.what() << std::endl;
            }
        }
        if (startupSummary) {
            std::cout << "[Map] map info updated, nodes=" << nodes.size()
                      << " edges=" << edges.size()
                      << " reserveNearConflictMm=" << nearConflictThresholdMm
                      << std::endl;
        }
        if (hasMapId) {
            mapId.store(parsedMapId);
            if (startupSummary) {
                std::cout << "[Map] mapId=" << parsedMapId << std::endl;
            }
        }
        const auto& rawEdgeList = edges;
        size_t sampleEdges = std::min<size_t>(5, rawEdgeList.size());
        if (startupSummary && sampleEdges > 0) {
            std::cout << "[Map] sample edges (first " << sampleEdges << "):" << std::endl;
            for (size_t i = 0; i < sampleEdges; ++i) {
                const auto& e = rawEdgeList[i];
                std::cout << "  #" << (i + 1) << " " << e.startNodeId
                          << " -> " << e.endNodeId
                          << " cost=" << e.weight << std::endl;
            }
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Map] failed to update map: " << e.what() << std::endl;
        return false;
    }
}

static std::optional<json> build_path_response_body(const std::string& messageId,
                                                    const std::string& deviceId,
                                                    const PlanCacheEntry& planEntry,
                                                    const MapInfo& mapInfo) {
    json resp;
    resp["messageId"] = messageId;
    resp["deviceId"] = deviceId;
    resp["code"] = 200;
    resp["error"] = "";
    json pathInfos = json::array();
    bool valid = true;
    for (const auto& seg : planEntry.plan.segments) {
        json info;
        json pathPoints = json::array();
        std::vector<int> route = seg.nodes;
        if (route.empty()) {
            if (seg.fromNodeId >= 0) route.push_back(seg.fromNodeId);
            if (seg.toNodeId >= 0 && (route.empty() || route.back() != seg.toNodeId)) {
                route.push_back(seg.toNodeId);
            }
        }
        if (!normalize_route_min_nodes(mapInfo, route, seg.fromNodeId, 1, false)) {
            valid = false;
            break;
        }
        PathPlanningHelper::PathSegmentInfo segResp = seg;
        segResp.nodes = route;
        segResp.fromNodeId = route.front();
        segResp.toNodeId = route.back();
        info["startPoint"] = build_segment_endpoint_point(mapInfo, segResp, true);
        info["endPoint"] = build_segment_endpoint_point(mapInfo, segResp, false);
        for (size_t i = 0; i < route.size(); ++i) {
            int nodeId = route[i];
            int yaw = resolve_point_yaw(mapInfo, route, i);
            json pt = make_path_point(mapInfo, nodeId, yaw);
            pathPoints.push_back(pt);
        }
        info["pathPoints"] = std::move(pathPoints);
        pathInfos.push_back(std::move(info));
    }
    const bool logSummary = env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL");
    if (!valid) {
        resp["code"] = 422;
        resp["error"] = "Invalid path response route";
        resp["pathInfos"] = json::array();
        return resp;
    }
    resp["pathInfos"] = std::move(pathInfos);
    if (logSummary) {
        std::cout << log_time_prefix() << "[PathResp] messageId=" << messageId
                  << " deviceId=" << deviceId
                  << " segments=" << resp["pathInfos"].size()
                  << std::endl;
    }
    if (env_enabled("RECEIVER_LOG_ROUTE_DETAIL")) {
        size_t printed = 0;
        for (const auto& info : resp["pathInfos"]) {
            const auto& pts = info["pathPoints"];
            for (size_t i = 0; i < pts.size(); ++i, ++printed) {
                std::cout << log_time_prefix() << "node[" << printed << "] = " << pts[i]["nodeId"]
                          << " , x=" << pts[i]["x"] << " , y=" << pts[i]["y"]
                          << std::endl;
            }
        }
    }
    return resp;
}

static std::string resolve_segment_subtask_id(const PathPlanningHelper::PathSegmentInfo& seg);

static std::optional<int> find_subtask_target_node(const PathPlanningHelper::AmrPlanInfo& plan,
                                                   const std::string& subTaskId) {
    if (subTaskId.empty()) return std::nullopt;
    for (const auto& seg : plan.segments) {
        const std::string segSubTaskId = resolve_segment_subtask_id(seg);
        if (!segSubTaskId.empty() && segSubTaskId == subTaskId) {
            return seg.toNodeId;
        }
    }
    return std::nullopt;
}

static int find_route_index_at_or_after(const std::vector<int>& route,
                                        int nodeId,
                                        size_t startIndex) {
    if (nodeId < 0 || route.empty()) return -1;
    if (startIndex >= route.size()) startIndex = route.size() - 1;
    for (size_t i = startIndex; i < route.size(); ++i) {
        if (route[i] == nodeId) return static_cast<int>(i);
    }
    return -1;
}

static bool digits_only(const std::string& s) {
    if (s.empty()) return false;
    for (unsigned char c : s) {
        if (!std::isdigit(c)) return false;
    }
    return true;
}

static std::optional<long long> try_parse_device_id_int64(const std::string& deviceId) {
    if (!digits_only(deviceId)) return std::nullopt;
    try {
        size_t idx = 0;
        long long v = std::stoll(deviceId, &idx, 10);
        if (idx != deviceId.size()) return std::nullopt;
        return v;
    } catch (...) {
        return std::nullopt;
    }
}

static bool device_id_less(const std::string& a, const std::string& b) {
    if (a == b) return false;
    auto ai = try_parse_device_id_int64(a);
    auto bi = try_parse_device_id_int64(b);
    if (ai.has_value() && bi.has_value()) return *ai < *bi;
    if (ai.has_value() && !bi.has_value()) return true;
    if (!ai.has_value() && bi.has_value()) return false;
    return a < b;
}

static ordered_json build_traffic_node_point(const MapInfo& mapInfo,
                                             int nodeId,
                                             int angleDeg,
                                             int pointType);

static ordered_json build_traffic_path_point(const MapInfo& mapInfo,
                                             int nodeId,
                                             int yawDeg) {
    ordered_json pt;
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

static ordered_json build_single_point_path_info(const std::string& deviceId,
                                                 int nodeId,
                                                 int angleDeg,
                                                 long long pathId,
                                                 const MapInfo& mapInfo) {
    ordered_json info;
    info["deviceId"] = deviceId;
    info["pathId"] = pathId;
    ordered_json start = build_traffic_node_point(mapInfo, nodeId, angleDeg, -1);
    info["startPoint"] = start;
    info["endPoint"] = start;
    ordered_json pathPoints = ordered_json::array();
    pathPoints.push_back(build_traffic_path_point(mapInfo, nodeId, angleDeg));
    info["pathPoints"] = std::move(pathPoints);
    info["subTaskId"] = "";
    info["taskPriority"] = 0;
    return info;
}

static ordered_json build_traffic_node_point(const MapInfo& mapInfo,
                                             int nodeId,
                                             int angleDeg,
                                             int pointType) {
    ordered_json pt;
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

static ordered_json build_traffic_segment_endpoint(const MapInfo& mapInfo,
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
    int angle = resolve_point_yaw(mapInfo, route, idx);
    int pointType = startPoint ? -1 : parse_step_type_code(seg.stepType);
    return build_traffic_node_point(mapInfo, route[idx], angle, pointType);
}

static std::string resolve_segment_subtask_id(const PathPlanningHelper::PathSegmentInfo& seg) {
    if (!seg.subTaskId.empty()) return seg.subTaskId;
    if (seg.taskId.empty()) return std::string();
    int seq = seg.subTaskSequence > 0 ? seg.subTaskSequence : 0;
    if (seq > 0) {
        return seg.taskId + "#" + std::to_string(seq);
    }
    return std::string();
}

static std::string make_traffic_route_key(const std::string& subTaskId,
                                          const std::string& taskId,
                                          int toNodeId,
                                          size_t orderHint) {
    if (!subTaskId.empty()) {
        return "sub:" + subTaskId;
    }
    std::ostringstream oss;
    if (!taskId.empty()) {
        oss << "task:" << taskId << "|";
    }
    oss << "to:" << toNodeId << "|idx:" << orderHint;
    return oss.str();
}

static ordered_json build_traffic_path_payload(
    const std::string& messageId,
    int mapId,
    RobotDataRepository& repo,
    const MapInfo& mapInfo,
    const std::unordered_map<std::string, int>& taskPriorityById,
    bool mapReady,
    bool staticTableReady,
    bool skipStaticTable,
    StaticPathTable& staticTable,
    AStarPathFinder* aStarPtr,
    NodeReservationTable& reservations
) {
    ordered_json body;
    body["messageId"] = messageId;
    body["code"] = 200;
    body["error"] = "";
    body["mapId"] = mapId;

    ordered_json pathInfos = ordered_json::array();
    auto planSnapshots = repo.snapshotPlans();
    std::sort(planSnapshots.begin(), planSnapshots.end(),
              [](const auto& a, const auto& b) { return device_id_less(a.first, b.first); });

    std::unordered_set<std::string> plannedIds;
    plannedIds.reserve(planSnapshots.size());

    const bool enableDynamic = mapReady && aStarPtr;
    const int trafficReserveBudget = std::max(1, getenv_int("TRAIL_MAX_POINTS", 10));
    const bool trafficSendGrowing = env_enabled("TRAFFIC_PATH_SEND_GROWING");

    for (const auto& kv : planSnapshots) {
        const std::string& deviceId = kv.first;
        const PlanCacheEntry& entry = kv.second;
        if (entry.plan.segments.empty()) {
            continue;
        }
        PathPlanningHelper::AmrPlanInfo planForTraffic = entry.plan;
        int currentNode = planForTraffic.startNodeId;
        long long trafficPathId = entry.pathId > 0 ? entry.pathId : current_millis();
        if (enableDynamic) {
            auto stOpt = repo.getStatusById(deviceId);
            if (stOpt.has_value()) {
                int resolvedNode = resolve_status_node_id(*stOpt, mapInfo);
                if (resolvedNode >= 0) currentNode = resolvedNode;
            }
            if (currentNode >= 0) {
                DynamicPlanCacheEntry dynEntry;
                if (repo.getDynamicPlanEntry(deviceId, entry.pathId, trafficReserveBudget, dynEntry)) {
                    planForTraffic = std::move(dynEntry.plan);
                    if (dynEntry.pathId > 0) {
                        trafficPathId = dynEntry.pathId;
                    }
                } else if (!planForTraffic.segments.empty()) {
                    trim_dynamic_plan_to_start(planForTraffic, currentNode, mapInfo);
                }
            }
        }
        if (planForTraffic.segments.empty()) {
            std::ostringstream oss;
            oss << "plan failed deviceId=" << deviceId
                << " start=" << planForTraffic.startNodeId
                << " reason=empty_segments";
            log_line(LogLevel::INFO, oss.str());
            continue;
        }
        long long pathId = trafficPathId;
        bool addedSegment = false;
        auto add_path_info = [&](const std::vector<int>& rawRoute,
                                 int targetNodeId,
                                 const std::string& subTaskId,
                                 const std::string& stepType,
                                 const std::string& taskId,
                                 const std::string& routeKey) -> bool {
            ordered_json info;
            info["deviceId"] = deviceId;
            info["pathId"] = pathId;
            info["subTaskId"] = subTaskId;
            int priority = 0;
            auto pit = entry.taskPriorities.find(taskId);
            if (pit != entry.taskPriorities.end()) {
                priority = pit->second;
            } else {
                auto fit = taskPriorityById.find(taskId);
                if (fit != taskPriorityById.end()) priority = fit->second;
            }
            info["taskPriority"] = priority;

            std::vector<int> route = rawRoute;
            if (!route.empty()) {
                if (!normalize_route_min_nodes(mapInfo, route, route.front(), 1, false)) {
                    return false;
                }
                std::vector<int> routeForPayload = route;
                if (trafficSendGrowing) {
                    routeForPayload = repo.updateTrafficGrowingRoute(deviceId,
                                                                    pathId,
                                                                    routeKey,
                                                                    route,
                                                                    currentNode);
                    if (routeForPayload.empty()) {
                        routeForPayload = route;
                    }
                }

                int startYaw = resolve_point_yaw(mapInfo, routeForPayload, 0);
                int endYaw = resolve_point_yaw(mapInfo, routeForPayload, routeForPayload.size() - 1);
                int endPointType = (targetNodeId >= 0 && route.back() == targetNodeId)
                    ? parse_step_type_code(stepType)
                    : -1;
                info["startPoint"] = build_traffic_node_point(mapInfo, routeForPayload.front(), startYaw, -1);
                info["endPoint"] = build_traffic_node_point(mapInfo, routeForPayload.back(), endYaw, endPointType);
                ordered_json pathPoints = ordered_json::array();
                for (size_t i = 0; i < routeForPayload.size(); ++i) {
                    int nodeId = routeForPayload[i];
                    int yaw = resolve_point_yaw(mapInfo, routeForPayload, i);
                    pathPoints.push_back(build_traffic_path_point(mapInfo, nodeId, yaw));
                }
                info["pathPoints"] = std::move(pathPoints);
            } else {
                int markerNode = (targetNodeId >= 0) ? targetNodeId : currentNode;
                int yaw = 0;
                if (markerNode >= 0) {
                    std::vector<int> tmp{markerNode};
                    yaw = resolve_point_yaw(mapInfo, tmp, 0);
                }
                int pointType = parse_step_type_code(stepType);
                info["startPoint"] = build_traffic_node_point(mapInfo, markerNode, yaw, -1);
                info["endPoint"] = build_traffic_node_point(mapInfo, markerNode, yaw, pointType);
                info["pathPoints"] = ordered_json::array();
            }
            pathInfos.push_back(std::move(info));
            return true;
        };

        if (enableDynamic) {
            struct TrafficSubtaskMarker {
                std::string subTaskId;
                int nodeId = -1;
                std::string stepType;
                std::string taskId;
            };
            std::vector<TrafficSubtaskMarker> markers;
            markers.reserve(planForTraffic.segments.size());
            for (const auto& seg : planForTraffic.segments) {
                if (seg.toNodeId < 0) continue;
                std::string subId = resolve_segment_subtask_id(seg);
                if (subId.empty()) continue;
                TrafficSubtaskMarker mk;
                mk.subTaskId = std::move(subId);
                mk.nodeId = seg.toNodeId;
                mk.stepType = seg.stepType;
                mk.taskId = seg.taskId;
                markers.push_back(std::move(mk));
            }
            if (!markers.empty()) {
                std::vector<int> windowRoute = build_route_from_plan(planForTraffic);
                if (!windowRoute.empty()) {
                    if (!normalize_route_min_nodes(mapInfo, windowRoute, currentNode, 1, false)) {
                        windowRoute.clear();
                    }
                }
                size_t cursor = 0;
                for (const auto& mk : markers) {
                    std::vector<int> segRoute;
                    if (!windowRoute.empty() && cursor < windowRoute.size()) {
                        int idx = find_route_index_at_or_after(windowRoute, mk.nodeId, cursor);
                        if (idx >= 0) {
                            segRoute.assign(windowRoute.begin() + static_cast<long long>(cursor),
                                            windowRoute.begin() + idx + 1);
                            cursor = static_cast<size_t>(idx);
                        } else {
                            segRoute.assign(windowRoute.begin() + static_cast<long long>(cursor),
                                            windowRoute.end());
                            cursor = windowRoute.size();
                        }
                    }
                    const std::string routeKey = make_traffic_route_key(
                        mk.subTaskId,
                        mk.taskId,
                        mk.nodeId,
                        cursor);
                    if (add_path_info(segRoute,
                                      mk.nodeId,
                                      mk.subTaskId,
                                      mk.stepType,
                                      mk.taskId,
                                      routeKey)) {
                        addedSegment = true;
                    }
                }
                if (addedSegment) {
                    plannedIds.insert(deviceId);
                }
                continue;
            }
        }

        if (enableDynamic && planForTraffic.segments.size() > 1) {
            planForTraffic.segments.resize(1);
        }
        const bool onlyFirstSegment = enableDynamic;
        size_t segOrder = 0;
        for (const auto& seg : planForTraffic.segments) {
            std::vector<int> route = seg.nodes;
            if (route.empty()) {
                if (seg.fromNodeId >= 0) route.push_back(seg.fromNodeId);
                if (seg.toNodeId >= 0 && (route.empty() || route.back() != seg.toNodeId)) {
                    route.push_back(seg.toNodeId);
                }
            }
            const std::string subTaskId = resolve_segment_subtask_id(seg);
            const std::string routeKey = make_traffic_route_key(
                subTaskId,
                seg.taskId,
                seg.toNodeId,
                segOrder++);
            if (add_path_info(route,
                              seg.toNodeId,
                              subTaskId,
                              seg.stepType,
                              seg.taskId,
                              routeKey)) {
                addedSegment = true;
            }
            if (onlyFirstSegment) {
                break;
            }
        }
        if (addedSegment) {
            plannedIds.insert(deviceId);
        }
    }

    auto statusSnapshots = repo.snapshotStatuses();
    std::sort(statusSnapshots.begin(), statusSnapshots.end(),
              [](const RobotStatusEntry& a, const RobotStatusEntry& b) {
                  return device_id_less(a.deviceId, b.deviceId);
              });
    for (const auto& st : statusSnapshots) {
        if (st.deviceId.empty()) continue;
        if (plannedIds.find(st.deviceId) != plannedIds.end()) continue;
        int nodeId = resolve_status_node_id(st, mapInfo);
        if (nodeId < 0) continue;
        int angleDeg = static_cast<int>(std::lround(st.angle)) % 360;
        if (angleDeg < 0) angleDeg += 360;
        long long pathId = repo.ensureFallbackPathId(st.deviceId, nodeId, current_millis());
        pathInfos.push_back(build_single_point_path_info(st.deviceId, nodeId, angleDeg, pathId, mapInfo));
    }

    body["pathInfos"] = std::move(pathInfos);
    return body;
}

static json build_trail_response_body(const std::string& messageId,
                                      const std::string& deviceId,
                                      const PlanCacheEntry& planEntry,
                                      const MapInfo& mapInfo,
                                      double intervalMm,
                                      int anchorNode) {
    (void)intervalMm;
    json resp;
    resp["messageId"] = messageId;
    resp["deviceId"] = deviceId;
    resp["code"] = 200;
    resp["error"] = "";
    json controlPoints = json::array();
    auto route = build_route_from_plan(planEntry.plan);
    if (!route.empty() && anchorNode >= 0 && route.front() != anchorNode) {
        // Avoid creating "anchor -> ... -> anchor" loops when we intentionally return a suffix
        // like [prev, anchor] to satisfy downstream minimum-point contracts.
        if (route.back() != anchorNode) {
            auto it = std::find(route.begin(), route.end(), anchorNode);
            if (it != route.end()) {
                route.erase(route.begin(), it);
            } else {
                route.insert(route.begin(), anchorNode);
            }
        }
    }
    if (route.empty()) route.push_back(planEntry.plan.startNodeId);
    int maxPoints = std::max(1, getenv_int("TRAIL_MAX_POINTS", 10));
    if ((int)route.size() > maxPoints) route.resize(maxPoints);

    for (size_t i = 0; i < route.size(); ++i) {
        int yaw = resolve_point_yaw(mapInfo, route, i);
        controlPoints.push_back(make_path_point(mapInfo, route[i], yaw));
    }
    resp["controlPoints"] = std::move(controlPoints);
    const bool logSummary = env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL");
    if (logSummary) {
        std::cout << "[TrailResp] messageId=" << messageId
                  << " deviceId=" << deviceId
                  << " points=" << resp["controlPoints"].size()
                  << std::endl;
    }
    if (env_enabled("RECEIVER_LOG_TRAIL_POINTS") && !env_enabled("RECEIVER_LOG_ROUTE_DETAIL")) {
        size_t maxPrint = 200;
        if (const char* limEnv = std::getenv("RECEIVER_LOG_TRAIL_POINTS_MAX")) {
            try { maxPrint = std::max<size_t>(1, static_cast<size_t>(std::stol(limEnv))); } catch (...) {}
        }
        const auto& cps = resp["controlPoints"];
        for (size_t i = 0; i < cps.size() && i < maxPrint; ++i) {
            const auto& pt = cps[i];
            std::cout << "  trailPoint[" << i << "] nodeId=" << read_string(pt, "nodeId", "-1")
                      << " x=" << pt.value("x", 0)
                      << " y=" << pt.value("y", 0)
                      << std::endl;
        }
        if (cps.size() > maxPrint) {
            std::cout << "  ... (" << cps.size() << " points total, truncated log at "
                      << maxPrint << ")" << std::endl;
        }
    }
    if (env_enabled("RECEIVER_LOG_ROUTE_DETAIL")) {
        constexpr size_t kMaxLogTrailPoints = 50;
        size_t printed = 0;
        const auto& cps = resp["controlPoints"];
        for (size_t i = 0; i < cps.size() && printed < kMaxLogTrailPoints; ++i, ++printed) {
            std::cout << "  controlPoint[" << printed << "]=" << cps[i]["nodeId"]
                      << " (x=" << cps[i]["x"] << ", y=" << cps[i]["y"] << ")"
                      << std::endl;
        }
    }
    return resp;
}

	static std::optional<json> build_trail_response_for_device(
	    const std::string& messageId,
	    const std::string& deviceId,
	    const std::string& subTaskId,
	    const std::optional<PlanCacheEntry>& planOpt,
	    RobotDataRepository& robotRepo,
	    const MapInfo& mapInfo,
	    bool mapReady,
	    bool skipStaticTable,
	    bool staticTableReady,
	    StaticPathTable& staticTable,
	    AStarPathFinder* aStarPtr,
	    NodeReservationTable& nodeReservations,
	    bool logSummary,
	    bool logDetail) {
	    json resp;
	    bool skipTrailResponse = false;
	    bool holdOnlyResponse = false;
	    std::vector<int> holdOnlyRoute;
	    if (planOpt && !deviceId.empty()) {
	        auto planCopy = *planOpt;
        if (logDetail) {
            std::cout << log_time_prefix() << "[TrailReq] planPathId=" << planCopy.pathId
                      << " schedId=" << (planCopy.schedulingRequestId.empty() ? "<none>" : planCopy.schedulingRequestId)
                      << " genTime=" << (planCopy.generatedTime.empty() ? "<none>" : planCopy.generatedTime)
                      << " segments=" << planCopy.plan.segments.size()
                      << std::endl;
        }
        int trailMaxPoints = std::max(1, getenv_int("TRAIL_MAX_POINTS", 10));
        int trailMinPoints = std::max(2, getenv_int("TRAIL_MIN_POINTS", 2));
        if (trailMaxPoints < trailMinPoints) trailMaxPoints = trailMinPoints;
        int currentNode = planCopy.plan.startNodeId;
        int committedNextNode = -1;
        int batteryLevel = 100;
        int deviceType = 0;
        auto stOpt = robotRepo.getStatusById(deviceId);
        if (stOpt.has_value()) {
            currentNode = resolve_status_node_id(*stOpt, mapInfo);
            committedNextNode = resolve_committed_next_node_id(*stOpt, mapInfo);
            batteryLevel = stOpt->batteryLevel;
            deviceType = stOpt->deviceType;
        }
        if (logDetail) {
            std::cout << log_time_prefix() << "[TrailReq] status currentNode=" << currentNode
                      << " committedNextNode=" << committedNextNode
                      << " battery=" << batteryLevel
                      << " type=" << deviceType
                      << std::endl;
        }
        bool hasDynamic = false;
        if (currentNode >= 0 && mapReady && aStarPtr) {
            PathPlanningHelper::AmrPlanInfo dyn;
            if (robotRepo.getDynamicPlanLatest(deviceId, planCopy.pathId, trailMaxPoints, dyn)) {
                planCopy.plan = std::move(dyn);
                hasDynamic = true;
            } else if (!planCopy.plan.segments.empty()) {
                trim_dynamic_plan_to_start(planCopy.plan, currentNode, mapInfo);
            }
        }
        if (!hasDynamic && planCopy.plan.segments.empty() && currentNode >= 0) {
            PathPlanningHelper::AmrPlanInfo idle;
            idle.amrId = deviceId;
            idle.startNodeId = currentNode;
            PathPlanningHelper::PathSegmentInfo seg;
            seg.fromNodeId = currentNode;
            seg.toNodeId = currentNode;
            seg.nodes = {currentNode};
            seg.stepType = "-1";
            seg.reachable = true;
            idle.segments.push_back(std::move(seg));
            planCopy.plan = std::move(idle);
        }
        auto route = build_route_from_plan(planCopy.plan);
        if (!deviceId.empty()) {
            PlanRuntimeState rt = robotRepo.snapshotRuntimeState(deviceId);
            if (!rt.reservedNodes.empty()) {
                std::vector<int> reservedRoute = rt.reservedNodes;
                if (currentNode >= 0) {
                    auto it = std::find(reservedRoute.begin(), reservedRoute.end(), currentNode);
                    if (it != reservedRoute.end()) {
                        if (it != reservedRoute.begin()) {
                            reservedRoute.erase(reservedRoute.begin(), it);
                        }
                    } else {
                        reservedRoute.insert(reservedRoute.begin(), currentNode);
                    }
                }
                if (!reservedRoute.empty()) {
                    route = std::move(reservedRoute);
                    if (logDetail) {
                        std::cout << log_time_prefix() << "[TrailResp] using reserved route head="
                                  << format_nodes_head(route) << std::endl;
                    }
                }
            }
        }
        if (route.empty() && planCopy.plan.startNodeId >= 0) {
            route.push_back(planCopy.plan.startNodeId);
            if (!planCopy.plan.segments.empty() && planCopy.plan.segments.front().toNodeId >= 0) {
                route.push_back(planCopy.plan.segments.front().toNodeId);
            }
        }
        std::optional<RobotStatusEntry> statusSnapshot;
        int anchorNode = -1;
        if (!deviceId.empty()) {
            statusSnapshot = robotRepo.getStatusById(deviceId);
            if (statusSnapshot) {
                anchorNode = resolve_status_node_id(*statusSnapshot, mapInfo);
            }
        }
        auto log_route_prefix = [&](const std::vector<int>& nodes) {
            if (logDetail) {
                std::cout << log_time_prefix() << "[TrailResp] globalRoute head=";
                if (nodes.empty()) {
                    std::cout << "<empty>";
                } else {
                    size_t limit = std::min<size_t>(5, nodes.size());
                    for (size_t i = 0; i < limit; ++i) {
                        if (i > 0) std::cout << " -> ";
                        std::cout << nodes[i];
                    }
                    if (nodes.size() > limit) std::cout << " ...";
                }
                std::cout << std::endl;
            }
        };
        if (logDetail) {
            if (statusSnapshot) {
                const auto& st = *statusSnapshot;
                std::cout << log_time_prefix() << "[TrailResp] anchor deviceId=" << deviceId
                          << " statusNode=" << (st.nodeId.empty() ? "<none>" : st.nodeId)
                          << " pos=(" << static_cast<int>(std::lround(st.x))
                          << "," << static_cast<int>(std::lround(st.y))
                          << ") anchorNode=" << anchorNode << std::endl;
            } else {
                std::cout << log_time_prefix() << "[TrailResp] anchor deviceId=" << deviceId
                          << " statusNode=<missing>"
                          << " anchorNode=" << anchorNode << std::endl;
            }
        }
        log_route_prefix(route);
        size_t startIndex = 0;
        if (!route.empty()) {
            size_t anchorHint = robotRepo.getAnchorProgress(deviceId);
            size_t searchHint = anchorHint;
            auto locateIndex = [&](int nodeId, size_t hint) -> int {
                if (nodeId < 0) return -1;
                int forward = find_route_index_at_or_after(route, nodeId, hint);
                if (forward >= 0) return forward;
                return find_route_index_at_or_after(route, nodeId, 0);
            };
            int idx = locateIndex(anchorNode, searchHint);
            if (idx < 0 && !route.empty()) {
                idx = locateIndex(route.front(), 0);
            }
            if (idx >= 0) {
                startIndex = static_cast<size_t>(idx);
                robotRepo.updateAnchorProgress(deviceId, startIndex);
            } else if (!route.empty() && startIndex >= route.size()) {
                startIndex = route.size() - 1;
            }
        }
        size_t maxPoints = static_cast<size_t>(trailMaxPoints);
        size_t minPoints = static_cast<size_t>(trailMinPoints);
        if (maxPoints < minPoints) maxPoints = minPoints;
        int targetNodeId = -1;
        if (!subTaskId.empty()) {
            auto target = find_subtask_target_node(planOpt->plan, subTaskId);
            if (!target.has_value()) {
                target = find_subtask_target_node(planCopy.plan, subTaskId);
            }
	            if (target.has_value()) {
	                targetNodeId = *target;
	            } else {
	                if (should_log_trail_missing_subtask(deviceId, subTaskId, logDetail)) {
	                    std::cout << log_time_prefix() << "[TrailResp] messageId=" << (messageId.empty() ? "<none>" : messageId)
	                              << " deviceId=" << (deviceId.empty() ? "<none>" : deviceId)
	                              << " subTaskId=" << subTaskId
	                              << " 未在计划中找到，对应节点未知，仍按默认长度返回" << std::endl;
	                }
	            }
	        }
        std::vector<int> trimmed;
        if (!route.empty()) {
            size_t remainingLen = (startIndex < route.size()) ? (route.size() - startIndex) : 0;
            size_t endIndex = std::min(route.size(), startIndex + maxPoints);
                if (logDetail) {
                    std::cout << log_time_prefix() << "[TrailReq] routeLen=" << route.size()
                              << " startIndex=" << startIndex
                              << " maxPoints=" << maxPoints
                              << " targetNodeId=" << targetNodeId
                              << std::endl;
                }
            if (targetNodeId >= 0) {
                size_t subHint = robotRepo.getSubTaskProgress(deviceId, subTaskId);
                if (subHint < startIndex) subHint = startIndex;
                int idx = find_route_index_at_or_after(route, targetNodeId, subHint);
                if (idx < 0 && subHint > startIndex) {
                    idx = find_route_index_at_or_after(route, targetNodeId, startIndex);
                }
                if (idx < 0 && startIndex > 0) {
                    idx = find_route_index_at_or_after(route, targetNodeId, 0);
                    if (idx >= 0) {
                        if (env_enabled("RECEIVER_LOG_ROUTE_DETAIL")) {
                            std::cout << "[TrailResp] subTaskId=" << subTaskId
                                      << " 重新从路径起点定位节点" << std::endl;
                        }
                    }
                }
                if (idx >= 0) {
                    endIndex = static_cast<size_t>(idx + 1);
                    robotRepo.updateSubTaskProgress(deviceId, subTaskId,
                                                    static_cast<size_t>(idx));
                } else {
                    if (env_enabled("RECEIVER_LOG_ROUTE_DETAIL")) {
                        std::cout << "[TrailResp] subTaskId=" << subTaskId
                                  << " 对应节点在当前路径上找不到，仍按默认长度截取" << std::endl;
                    }
                }
            }
            if (endIndex <= startIndex) {
                endIndex = std::min(route.size(), startIndex + 1);
            }
            trimmed.assign(route.begin() + startIndex, route.begin() + endIndex);
            if (remainingLen <= 1) {
                if (committedNextNode >= 0 && !trimmed.empty()
                           && committedNextNode != trimmed.front()) {
                    trimmed.push_back(committedNextNode);
                }
            }
            if (!skipTrailResponse) {
                bool insertedAnchor = false;
                if (anchorNode >= 0) {
                    const bool startsAtAnchor = !trimmed.empty() && trimmed.front() == anchorNode;
                    const bool holdEndsAtAnchor = trimmed.size() >= 2 && trimmed.back() == anchorNode;
                    if (!startsAtAnchor && !holdEndsAtAnchor) {
                        trimmed.insert(trimmed.begin(), anchorNode);
                        insertedAnchor = true;
                    }
                }
                if (trimmed.empty()) {
                    if (anchorNode >= 0) trimmed.push_back(anchorNode);
                    else if (planCopy.plan.startNodeId >= 0) trimmed.push_back(planCopy.plan.startNodeId);
                    else trimmed.push_back(-1);
                }
                if (insertedAnchor && trimmed.size() > maxPoints) {
                    trimmed.erase(trimmed.begin() + 1);
                } else if (trimmed.size() > maxPoints) {
                    trimmed.resize(maxPoints);
                }
                size_t committedRequired = 0;
                std::vector<int> lastSent = robotRepo.getLastSentRoute(deviceId);
                if (!lastSent.empty()) {
                    size_t hint = robotRepo.getAnchorProgress(deviceId);
                    int idx = find_route_index_at_or_after(lastSent, currentNode, hint);
                    if (idx < 0) {
                        idx = find_route_index_at_or_after(lastSent, currentNode, 0);
                    }
                    if (idx < 0) {
                        if (logSummary) {
                            std::cout << log_time_prefix() << "[TrailResp] skip rollback (anchor missing) deviceId="
                                      << deviceId << std::endl;
                        }
                        skipTrailResponse = true;
                    } else {
                        std::vector<int> remaining(lastSent.begin() + idx, lastSent.end());
                        committedRequired = remaining.size();
                        bool prefixOk = remaining.size() <= trimmed.size()
                            && std::equal(remaining.begin(), remaining.end(), trimmed.begin());
                        if (!prefixOk && !remaining.empty()) {
                            bool trimmedIsPrefix = trimmed.size() <= remaining.size()
                                && std::equal(trimmed.begin(), trimmed.end(), remaining.begin());
                            if (logSummary) {
                                std::cout << log_time_prefix()
                                          << (trimmedIsPrefix
                                              ? "[TrailResp] extend committed prefix deviceId="
                                              : "[TrailResp] force committed prefix deviceId=")
                                          << deviceId
                                          << " currentNode=" << currentNode
                                          << " lastSentRem=" << format_nodes_head(remaining)
                                          << " trimmed=" << format_nodes_head(trimmed)
                                          << " lastSentSize=" << lastSent.size()
                                          << " trimmedSize=" << trimmed.size()
                                          << std::endl;
                            }
                            trimmed = remaining;
                            prefixOk = true;
                        }
                        if (!prefixOk) {
                            if (logSummary) {
                                std::cout << log_time_prefix() << "[TrailResp] skip rollback (prefix mismatch) deviceId="
                                          << deviceId
                                          << " currentNode=" << currentNode
                                          << " lastSentRem=" << format_nodes_head(remaining)
                                          << " trimmed=" << format_nodes_head(trimmed)
                                          << " lastSentSize=" << lastSent.size()
                                          << " trimmedSize=" << trimmed.size()
                                          << std::endl;
                            }
                            skipTrailResponse = true;
                        }
                    }
                }
	                if (!skipTrailResponse && trimmed.size() < minPoints) {
	                    if (logSummary) {
	                        std::cout << log_time_prefix() << "[TrailResp] skip short trail deviceId="
	                                  << deviceId
	                                  << " size=" << trimmed.size()
	                                  << " min=" << minPoints
	                                  << std::endl;
	                    }
	                    // If we are not committed to a longer prefix, respond with a single-point
	                    // "hold" trail instead of silently dropping the response (prevents requester stalls).
	                    if (committedRequired <= 1) {
	                        int holdNode = (anchorNode >= 0) ? anchorNode : currentNode;
	                        if (holdNode < 0) holdNode = planCopy.plan.startNodeId;
	                        if (holdNode >= 0) {
	                            holdOnlyResponse = true;
	                            holdOnlyRoute.assign(1, holdNode);
	                        }
	                    }
	                    skipTrailResponse = true;
	                }
                if (!skipTrailResponse) {
                    bool invalidNode = false;
                    for (int nodeId : trimmed) {
                        if (nodeId < 0) {
                            invalidNode = true;
                            break;
                        }
                    }
                    if (invalidNode) {
                        if (logSummary) {
                            std::cout << log_time_prefix() << "[TrailResp] skip short/invalid route deviceId="
                                      << deviceId
                                      << " size=" << trimmed.size()
                                      << " min=" << minPoints
                                      << " invalid=" << (invalidNode ? "1" : "0")
                                      << std::endl;
                        }
                        skipTrailResponse = true;
                    }
                }
                if (skipTrailResponse) {
                    // Do not rollback: consume message without sending.
                    trimmed.clear();
                } else {
                    planCopy.plan.segments.clear();
                    PathPlanningHelper::PathSegmentInfo seg;
                    seg.nodes = trimmed;
                    seg.fromNodeId = trimmed.front();
                    seg.toNodeId = trimmed.back();
                    planCopy.plan.segments.push_back(seg);
                    std::vector<int> committed;
                    committed.reserve(trimmed.size());
                    const int committedTtlMs = std::max(0, getenv_int("RESERVE_COMMITTED_TTL_MS", 0));
                    bool commitOk = true;
                    int commitFailNode = -1;
                    for (int nodeId : trimmed) {
                        if (nodeId < 0) continue;
                        int ttlMs = committedTtlMs;
                        if (nodeId == currentNode) {
                            ttlMs = 0;
                        }
                        if (!nodeReservations.tryReserve(nodeId,
                                                         deviceId,
                                                         NodeReservationTable::HoldReason::RESERVED_PATH,
                                                         "committed",
                                                         std::chrono::milliseconds(std::max(0, ttlMs)))) {
                            commitOk = false;
                            commitFailNode = nodeId;
                            break;
                        }
                        committed.push_back(nodeId);
                    }
                    if (!commitOk) {
                        if (committed.size() < committedRequired) {
                            if (logSummary) {
                                auto hold = nodeReservations.findBlockingHold(commitFailNode, deviceId);
                                std::cout << log_time_prefix()
                                          << "[TrailResp] skip commit failed deviceId=" << deviceId
                                          << " node=" << commitFailNode
                                          << " holder=" << (hold.has_value() ? hold->ownerAgvId : "<none>")
                                          << " reason=" << (hold.has_value() ? hold_reason_label(hold->reason) : "<none>")
                                          << " detail=" << (hold.has_value() ? hold->detail : "<none>")
                                          << std::endl;
                            }
                            skipTrailResponse = true;
                            trimmed.clear();
                        } else {
                            trimmed = std::move(committed);
                        }
                    } else {
                        trimmed = std::move(committed);
                    }
                }
            }
        } else {
            skipTrailResponse = true;
	        }
	        if (holdOnlyResponse && !holdOnlyRoute.empty()) {
	            // Build a minimal trail response (1 point) to acknowledge the request without rollback.
	            planCopy.plan.segments.clear();
	            PathPlanningHelper::PathSegmentInfo seg;
	            seg.nodes = holdOnlyRoute;
	            seg.fromNodeId = holdOnlyRoute.front();
	            seg.toNodeId = holdOnlyRoute.back();
	            seg.reachable = true;
	            planCopy.plan.segments.push_back(std::move(seg));
	            resp = build_trail_response_body(
	                messageId,
	                deviceId,
	                planCopy,
	                mapInfo,
	                100.0,
	                anchorNode);
	            // Do not update lastSentRoute: avoid shrinking committed prefix on a hold response.
	            skipTrailResponse = false;
	        } else if (!skipTrailResponse) {
	            resp = build_trail_response_body(
	                messageId,
	                deviceId,
	                planCopy,
	                mapInfo,
	                100.0,
	                anchorNode);
	            if (!trimmed.empty()) {
	                robotRepo.updateLastSentRoute(deviceId, trimmed);
	            }
	        }
    } else {
        if (!deviceId.empty()) {
            skipTrailResponse = true;
        } else {
            resp["messageId"] = messageId;
            resp["deviceId"] = deviceId;
            resp["code"] = 404;
            resp["error"] = "No cached plan for device";
            resp["controlPoints"] = json::array();
        }
    }
    if (skipTrailResponse) {
        if (logSummary) {
            std::cout << log_time_prefix() << "[TrailResp] skip single-point trail messageId="
                      << (messageId.empty() ? "<none>" : messageId)
                      << " deviceId=" << (deviceId.empty() ? "<none>" : deviceId)
                      << std::endl;
        }
        return std::nullopt;
    }
    return resp;
}

static void publish_trail_for_all_plans(
    const std::string& messageIdPrefix,
    RobotDataRepository& robotRepo,
    const MapInfo& mapInfo,
    std::shared_mutex& mapMutex,
    bool mapReady,
    bool skipStaticTable,
    bool staticTableReady,
    StaticPathTable& staticTable,
    AStarPathFinder* aStarPtr,
    NodeReservationTable& nodeReservations,
    AlgoPublisher& algoPublisher,
    bool logSummary,
    bool logDetail) {
    auto planSnapshots = robotRepo.snapshotPlans();
    if (planSnapshots.empty()) return;
    std::shared_lock<std::shared_mutex> mapReadLock(mapMutex);
    for (const auto& kv : planSnapshots) {
        const std::string& deviceId = kv.first;
        if (deviceId.empty()) continue;
        std::string msgId = messageIdPrefix;
        if (msgId.empty()) {
            msgId = "trail_" + deviceId + "_" + std::to_string(current_millis());
        } else {
            msgId = msgId + "_" + deviceId;
        }
        std::optional<PlanCacheEntry> planOpt = kv.second;
        auto respOpt = build_trail_response_for_device(
            msgId,
            deviceId,
            std::string(),
            planOpt,
            robotRepo,
            mapInfo,
            mapReady,
            skipStaticTable,
            staticTableReady,
            staticTable,
            aStarPtr,
            nodeReservations,
            logSummary,
            logDetail);
        if (respOpt.has_value()) {
            algoPublisher.sendTrailResponse(*respOpt);
        }
    }
}

static std::optional<PlanCacheEntry> wait_for_plan(
    RobotDataRepository& repo,
    const std::string& deviceId,
    int wait_ms) {
    if (deviceId.empty()) return std::nullopt;
    const int step_ms = 50;
    int elapsed = 0;
    while (elapsed < wait_ms) {
        if (auto plan = repo.getPlan(deviceId)) return plan;
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
        elapsed += step_ms;
    }
    return repo.getPlan(deviceId);
}

}  // namespace
static bool amqp_ok(amqp_rpc_reply_t r, const char* ctx) {
    switch (r.reply_type) {
        case AMQP_RESPONSE_NORMAL:
            return true;
        case AMQP_RESPONSE_NONE:
            std::cerr << "AMQP error in " << ctx << ": response none" << std::endl;
            break;
        case AMQP_RESPONSE_LIBRARY_EXCEPTION:
            std::cerr << "AMQP error in " << ctx << ": lib err="
                      << amqp_error_string2(r.library_error) << std::endl;
            break;
        case AMQP_RESPONSE_SERVER_EXCEPTION:
            if (r.reply.id == AMQP_CONNECTION_CLOSE_METHOD) {
                auto* m = (amqp_connection_close_t*)r.reply.decoded;
                std::cerr << "AMQP connection close in " << ctx
                          << " reply-code=" << m->reply_code
                          << " text="
                          << std::string((char*)m->reply_text.bytes, m->reply_text.len)
                          << std::endl;
            } else if (r.reply.id == AMQP_CHANNEL_CLOSE_METHOD) {
                auto* m = (amqp_channel_close_t*)r.reply.decoded;
                std::cerr << "AMQP channel close in " << ctx
                          << " reply-code=" << m->reply_code
                          << " text="
                          << std::string((char*)m->reply_text.bytes, m->reply_text.len)
                          << std::endl;
            } else {
                std::cerr << "AMQP server exception in " << ctx
                          << " method-id=" << r.reply.id << std::endl;
            }
            break;
    }
    return false;
}

// 环境变量读取工具
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

static bool env_enabled(const char* key) {
    if (const char* v = std::getenv(key)) {
        std::string s(v);
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
        return (s == "1" || s == "true" || s == "yes" || s == "on");
    }
    return false;
}

static bool env_enabled_default_true(const char* key) {
    if (const char* v = std::getenv(key)) {
        std::string s(v);
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
        if (s == "0" || s == "false" || s == "no" || s == "off") return false;
        if (s == "1" || s == "true" || s == "yes" || s == "on") return true;
    }
    return true;
}

// 确保无论何处 return 都能停止并回收心跳线程，避免 std::terminate
struct HeartbeatGuard {
    std::atomic<bool>& running;
    std::thread& th;
    ~HeartbeatGuard() {
        running = false;
        if (th.joinable()) th.join();
    }
};

struct InitTickerGuard {
    std::atomic<bool>& running;
    std::thread& th;
    ~InitTickerGuard() {
        running = false;
        if (th.joinable()) th.join();
    }
};

struct TrafficTickerGuard {
    std::atomic<bool>& running;
    std::thread& th;
    ~TrafficTickerGuard() {
        running = false;
        if (th.joinable()) th.join();
    }
};

struct ReplanTickerGuard {
    std::atomic<bool>& running;
    std::thread& th;
    ~ReplanTickerGuard() {
        running = false;
        if (th.joinable()) th.join();
    }
};

int main() {
    auto searchRoots = PathUtils::commonRoots();
    PathUtils::addRootIfSet(searchRoots, "AGV_SCHED_ROOT");
    std::atomic<bool> initSentAfterMap(false);
    g_trailVersion = detect_trail_version_mode();
    const bool startupSummary = env_enabled("RECEIVER_LOG_STARTUP_SUMMARY");
    if (startupSummary) {
        std::cout << "[Trail] version=" << trail_version_label(g_trailVersion)
                  << " periodic=" << (trail_periodic_enabled(g_trailVersion) ? "1" : "0")
                  << std::endl;
    }
    if (startupSummary) {
        std::cout << "ExternalReceiver: 启动时不加载本地地图，等待业务侧发送 SendMapInfo。" << std::endl;
    }
    std::string staticTablePath = getenv_str("PLANNER_STATIC_TABLE",
        "config/south_20260107_all.bin");
    staticTablePath = PathUtils::resolvePathOrWarn(
        staticTablePath,
        "static path table",
        searchRoots);
    // 默认加载静态表；可通过 SKIP_STATIC_TABLE=1 跳过
    bool defaultSkipStaticTable = env_enabled("SKIP_STATIC_TABLE");
    if (defaultSkipStaticTable) {
        if (startupSummary) {
            std::cout << "ExternalReceiver: SKIP_STATIC_TABLE=1，跳过静态路径表加载，直接使用 A*。" << std::endl;
        }
    } else {
        StaticPathTable staticTableProbe;
        if (!staticTableProbe.loadFromFile(staticTablePath)) {
            std::cerr << "ExternalReceiver: failed to load StaticPathTable "
                      << staticTablePath << std::endl;
            std::cerr << "ExternalReceiver: static table invalid, auto-skip to A*." << std::endl;
            defaultSkipStaticTable = true;
        }
    }
    MultiMapManager mapManager(defaultSkipStaticTable, staticTablePath);
    SchedulingTaskQueue mapReloadQueue;
    mapReloadQueue.start(1);
    std::atomic<int> schedulingBusyCount(0);

    RabbitMQConfig cfg = RabbitMQConfigFromEnv({});
    ResultPublisher resultPublisher(cfg);
    AlgoPublisher algoPublisher(cfg);
    TrafficPathPublisher trafficPathPublisher(cfg);
    TrafficDebugManager trafficDebug;
    std::string resultDumpDir;
    std::string unknownRkDumpDir;
    std::string serviceName = getenv_str("ALGO_SERVICE_NAME", "AgvSchedulingService");
    std::atomic<bool> heartbeatRunning(false);
    std::thread heartbeatThread;
    HeartbeatGuard hbGuard{heartbeatRunning, heartbeatThread};
    std::atomic<bool> initTickerRunning(false);
    std::thread initTickerThread;
    InitTickerGuard initGuard{initTickerRunning, initTickerThread};
    std::atomic<bool> trafficTickerRunning(false);
    std::thread trafficTickerThread;
    TrafficTickerGuard trafficGuard{trafficTickerRunning, trafficTickerThread};
    std::atomic<bool> replanTickerRunning(false);
    std::thread replanTickerThread;
    ReplanTickerGuard replanGuard{replanTickerRunning, replanTickerThread};
    SchedulingTaskQueue replanQueue;
    std::atomic<bool> replanQueueStarted(false);

    auto start_heartbeat = [&]() {
        heartbeatRunning.store(true);
        heartbeatThread = std::thread([&]() {
            while (heartbeatRunning.load()) {
                bool allocating = schedulingBusyCount.load() > 0;
                algoPublisher.sendHeartbeat(serviceName, allocating);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        });
    };

    auto send_initialize = [&]() {
        algoPublisher.sendServiceInitialize(serviceName);
        if (!heartbeatRunning.load()) start_heartbeat();
    };

    auto start_init_ticker = [&]() {
        if (initTickerRunning.exchange(true)) return;
        initTickerThread = std::thread([&]() {
            while (initTickerRunning.load()) {
                if (!mapManager.anyMapReady()) {
                    send_initialize();
                } else {
                    break;
                }
                for (int i = 0; i < 5; ++i) {
                    if (!initTickerRunning.load() || mapManager.anyMapReady()) break;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        });
    };
    auto start_traffic_ticker = [&]() {
        if (!traffic_periodic_enabled(g_trailVersion)) return;
        if (trafficTickerRunning.exchange(true)) return;
        trafficTickerThread = std::thread([&]() {
            TrafficPathPublisher tickerPublisher(cfg);
            std::unordered_map<std::string, int> emptyPriorities;
            while (trafficTickerRunning.load()) {
                int intervalMs = getenv_int("TRAFFIC_PATH_PUBLISH_INTERVAL_MS", 100);
                if (intervalMs <= 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    continue;
                }
                intervalMs = std::max(50, intervalMs);
                auto contexts = mapManager.snapshotContexts();
                for (const auto& ctx : contexts) {
                    if (!ctx->mapReady.load()) continue;
                    std::string tickId = "traffic_periodic_" + std::to_string(ctx->activeMapId.load())
                                       + "_" + std::to_string(current_millis());
                    ordered_json payload;
                    {
                        std::shared_lock<std::shared_mutex> mapReadLock(ctx->mapMutex);
                        payload = build_traffic_path_payload(
                            tickId,
                            ctx->activeMapId.load(),
                            ctx->robotRepo,
                            *ctx->mapInfoPtr,
                            emptyPriorities,
                            ctx->mapReady.load(),
                            ctx->staticTableReady,
                            ctx->skipStaticTable,
                            ctx->staticTable,
                            ctx->aStarPtr.get(),
                            ctx->nodeReservations);
                        if (!ctx->mapVersion.empty()) {
                            payload["mapVersion"] = ctx->mapVersion;
                        }
                    }
                    std::string payloadStr = payload.dump();
                    tickerPublisher.publishPayload(payloadStr);
                    trafficDebug.onPublish(payload, payloadStr, "periodic");
                }
                int stepMs = 50;
                int steps = std::max(1, intervalMs / stepMs);
                for (int i = 0; i < steps; ++i) {
                    if (!trafficTickerRunning.load()) break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(stepMs));
                }
            }
        });
    };
    auto start_replan_ticker = [&]() {
        if (replanTickerRunning.exchange(true)) return;
        replanTickerThread = std::thread([&]() {
            while (replanTickerRunning.load()) {
                const int tickMs = std::max(20, getenv_int("REPLAN_TICK_MS", 200));
                const int baseIntervalMs = tickMs;
                auto contexts = mapManager.snapshotContexts();
                bool hasReadyContext = false;
                for (const auto& ctx : contexts) {
                    if (ctx->mapReady.load() && ctx->aStarPtr) {
                        hasReadyContext = true;
                        break;
                    }
                }
                if (!hasReadyContext) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    continue;
                }
                const bool profileSummary = env_enabled("REPLAN_PROFILE");
                const int replanWorkers = std::max(1, getenv_int("REPLAN_WORKERS", 1));
                if (replanWorkers > 1 && !replanQueueStarted.load()) {
                    replanQueue.start(static_cast<size_t>(replanWorkers));
                    replanQueueStarted.store(true);
                    std::cout << "[Replan] workers=" << replanWorkers << std::endl;
                }
                std::vector<std::tuple<std::shared_ptr<MapRuntimeContext>, std::string, PlanCacheEntry>> tasks;
                for (const auto& ctx : contexts) {
                    auto planSnapshots = ctx->robotRepo.snapshotPlans();
                    for (const auto& kv : planSnapshots) {
                        if (kv.second.plan.segments.empty()) continue;
                        tasks.emplace_back(ctx, kv.first, kv.second);
                    }
                }
                if (!tasks.empty()) {
                    const auto tickStart = std::chrono::steady_clock::now();
                    const int reserveBudgetOverride = std::max(1, getenv_int("TRAIL_MAX_POINTS", 10));
                    long long totalMs = 0;
                    long long maxMs = 0;
                    std::string slowestId;
                    std::mutex statsMu;
                    auto record_stats = [&](const std::string& deviceId, long long costMs) {
                        if (!profileSummary) return;
                        std::lock_guard<std::mutex> lk(statsMu);
                        totalMs += costMs;
                        if (costMs > maxMs) {
                            maxMs = costMs;
                            slowestId = deviceId;
                        }
                    };
                    if (replanWorkers <= 1) {
                        int jobCount = 0;
                        for (const auto& task : tasks) {
                            const auto& ctx = std::get<0>(task);
                            const std::string& deviceId = std::get<1>(task);
                            const PlanCacheEntry& planEntry = std::get<2>(task);
                            const auto jobStart = std::chrono::steady_clock::now();
                            std::shared_lock<std::shared_mutex> mapReadLock(ctx->mapMutex);
                            if (!ctx->mapReady.load() || !ctx->aStarPtr) continue;
                            int currentNodeId = planEntry.plan.startNodeId;
                            int committedNextNodeId = -1;
                            int batteryLevel = 100;
                            int deviceType = 0;
                            auto stOpt = ctx->robotRepo.getStatusById(deviceId);
                            if (stOpt.has_value()) {
                                int resolvedNode = resolve_status_node_id(*stOpt, *ctx->mapInfoPtr);
                                if (resolvedNode >= 0) currentNodeId = resolvedNode;
                                committedNextNodeId = resolve_committed_next_node_id(*stOpt, *ctx->mapInfoPtr);
                                batteryLevel = stOpt->batteryLevel;
                                deviceType = stOpt->deviceType;
                            }
                            if (currentNodeId >= 0) {
                                jobCount += 1;
                                compute_dynamic_plan_with_throttle(
                                    deviceId,
                                    currentNodeId,
                                    committedNextNodeId,
                                    batteryLevel,
                                    deviceType,
                                    planEntry,
                                    ctx->robotRepo,
                                    *ctx->mapInfoPtr,
                                    ctx->skipStaticTable,
                                    ctx->staticTableReady,
                                    ctx->staticTable,
                                    *ctx->aStarPtr,
                                    ctx->nodeReservations,
                                    reserveBudgetOverride,
                                    baseIntervalMs,
                                    nullptr);
                            }
                            if (profileSummary) {
                                const auto jobMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - jobStart).count();
                                record_stats(deviceId, jobMs);
                            }
                        }
                        if (profileSummary && jobCount > 0) {
                            const auto tickMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - tickStart).count();
                            const long long avgMs = (jobCount > 0) ? (totalMs / jobCount) : 0;
                            std::cout << log_time_prefix()
                                      << "[ReplanProfile] tickMs=" << tickMs
                                      << " jobs=" << jobCount
                                      << " avgMs=" << avgMs
                                      << " maxMs=" << maxMs
                                      << " slowest=" << (slowestId.empty() ? "<none>" : slowestId)
                                      << std::endl;
                        }
                    } else {
                        const int jobCount = static_cast<int>(tasks.size());
                        if (jobCount > 0) {
                            std::atomic<int> remaining(jobCount);
                            std::mutex waitMu;
                            std::condition_variable waitCv;
                            for (const auto& task : tasks) {
                                const auto ctx = std::get<0>(task);
                                const std::string deviceId = std::get<1>(task);
                                const PlanCacheEntry planEntry = std::get<2>(task);
                                replanQueue.submit([&, ctx, deviceId, planEntry]() {
                                    const auto jobStart = std::chrono::steady_clock::now();
                                    if (ctx->mapReady.load()) {
                                        std::shared_lock<std::shared_mutex> mapReadLock(ctx->mapMutex);
                                        AStarPathFinder* aStarLocal = ctx->aStarPtr.get();
                                        if (aStarLocal && !planEntry.plan.segments.empty()) {
                                            int currentNodeId = planEntry.plan.startNodeId;
                                            int committedNextNodeId = -1;
                                            int batteryLevel = 100;
                                            int deviceType = 0;
                                            auto stOpt = ctx->robotRepo.getStatusById(deviceId);
                                            if (stOpt.has_value()) {
                                                int resolvedNode = resolve_status_node_id(*stOpt, *ctx->mapInfoPtr);
                                                if (resolvedNode >= 0) currentNodeId = resolvedNode;
                                                committedNextNodeId = resolve_committed_next_node_id(*stOpt, *ctx->mapInfoPtr);
                                                batteryLevel = stOpt->batteryLevel;
                                                deviceType = stOpt->deviceType;
                                            }
                                            if (currentNodeId >= 0) {
                                                compute_dynamic_plan_with_throttle(
                                                    deviceId,
                                                    currentNodeId,
                                                    committedNextNodeId,
                                                    batteryLevel,
                                                    deviceType,
                                                    planEntry,
                                                    ctx->robotRepo,
                                                    *ctx->mapInfoPtr,
                                                    ctx->skipStaticTable,
                                                    ctx->staticTableReady,
                                                    ctx->staticTable,
                                                    *aStarLocal,
                                                    ctx->nodeReservations,
                                                    reserveBudgetOverride,
                                                    baseIntervalMs,
                                                    nullptr);
                                            }
                                        }
                                    }
                                    if (profileSummary) {
                                        const auto jobMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                            std::chrono::steady_clock::now() - jobStart).count();
                                        record_stats(deviceId, jobMs);
                                    }
                                    if (remaining.fetch_sub(1) == 1) {
                                        std::lock_guard<std::mutex> lk(waitMu);
                                        waitCv.notify_one();
                                    }
                                });
                            }
                            std::unique_lock<std::mutex> lk(waitMu);
                            waitCv.wait(lk, [&]() {
                                return remaining.load() <= 0 || !replanTickerRunning.load();
                            });
                            if (profileSummary) {
                                const auto tickMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - tickStart).count();
                                const long long avgMs = (jobCount > 0) ? (totalMs / jobCount) : 0;
                                std::cout << log_time_prefix()
                                          << "[ReplanProfile] tickMs=" << tickMs
                                          << " jobs=" << jobCount
                                          << " avgMs=" << avgMs
                                          << " maxMs=" << maxMs
                                          << " slowest=" << (slowestId.empty() ? "<none>" : slowestId)
                                          << std::endl;
                            }
                        }
                    }
                }
                int stepMs = 50;
                int steps = std::max(1, tickMs / stepMs);
                for (int i = 0; i < steps; ++i) {
                    if (!replanTickerRunning.load()) break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(stepMs));
                }
            }
        });
    };
    // 若尚未收到地图，定期提醒业务端（每5s）服务已上线
    start_init_ticker();
    start_traffic_ticker();
    start_replan_ticker();

    SchedulingTaskQueue schedulingQueue;
    unsigned int hwThreads = std::thread::hardware_concurrency();
    int defaultWorkers = static_cast<int>(hwThreads == 0 ? 2 : hwThreads);
    int workerCount = getenv_int("SCHED_WORKERS", defaultWorkers);
    if (workerCount <= 0) workerCount = defaultWorkers;
    schedulingQueue.start(static_cast<size_t>(workerCount));
    std::cout << "[SchedulingQueue] workers=" << workerCount << std::endl;
    std::mutex schedulingLatestMutex;
    std::optional<json> schedulingLatest;
    bool schedulingWorkerActive = false;
    std::unique_ptr<SimPosePublisher> simPosePublisher;
    std::unique_ptr<MultiMapSimPoseStreamer> simPoseStreamer;
    std::unique_ptr<SimReservedPublisher> simReservedPublisher;
    std::unique_ptr<MultiMapSimReservedStreamer> simReservedStreamer;
    if (env_enabled("ENABLE_SIM_POSE_STREAM")) {
        try {
            int poseInterval = getenv_int("SIM_POSE_INTERVAL_MS", 200);
            simPosePublisher.reset(new SimPosePublisher(cfg));
            simPoseStreamer.reset(new MultiMapSimPoseStreamer(mapManager, *simPosePublisher, poseInterval));
            simPoseStreamer->start();
            if (startupSummary) {
                std::cout << "[SimPose] pose stream enabled interval=" << poseInterval << "ms" << std::endl;
            }
        } catch (const std::exception& ex) {
            std::cerr << "[SimPose] failed to start: " << ex.what() << std::endl;
            simPoseStreamer.reset();
            simPosePublisher.reset();
        }
    }
    if (env_enabled("ENABLE_SIM_RESERVED_STREAM")) {
        try {
            int reserveInterval = getenv_int("SIM_RESERVED_INTERVAL_MS", 200);
            simReservedPublisher.reset(new SimReservedPublisher(cfg));
            simReservedStreamer.reset(new MultiMapSimReservedStreamer(mapManager, *simReservedPublisher,
                                                                      reserveInterval));
            simReservedStreamer->start();
            if (startupSummary) {
                std::cout << "[SimReserved] stream enabled interval=" << reserveInterval << "ms" << std::endl;
            }
        } catch (const std::exception& ex) {
            std::cerr << "[SimReserved] failed to start: " << ex.what() << std::endl;
            simReservedStreamer.reset();
            simReservedPublisher.reset();
        }
    }
    {
        std::string dumpDirEnv = getenv_str("RESULT_DUMP_DIR", "debug");
        if (!dumpDirEnv.empty()) {
            resultDumpDir = PathUtils::resolvePathOrWarn(
                dumpDirEnv, "result dump directory", searchRoots);
            resultPublisher.setDumpDirectory(resultDumpDir);
            if (startupSummary) {
                std::cout << "ExternalReceiver: 将调度结果保存到 " << resultDumpDir
                          << std::endl;
            }
        } else {
            if (startupSummary) {
                std::cout << "ExternalReceiver: 结果落盘已关闭（RESULT_DUMP_DIR 为空）。"
                          << std::endl;
            }
        }
    }
    {
        std::string unknownDumpEnv = getenv_str("UNKNOWN_RK_DUMP_DIR", resultDumpDir.c_str());
        if (!unknownDumpEnv.empty()) {
            unknownRkDumpDir = PathUtils::resolvePathOrWarn(
                unknownDumpEnv, "unknown rk dump directory", searchRoots);
        } else {
            unknownRkDumpDir.clear();
        }
    }
    {
        int trafficDumpKeepMax = 0;
        bool trafficDumpKeepSet = false;
        if (const char* keepEnv = std::getenv("TRAFFIC_DUMP_KEEP_MAX")) {
            trafficDumpKeepSet = true;
            try {
                trafficDumpKeepMax = std::stoi(keepEnv);
            } catch (...) {
                trafficDumpKeepMax = 0;
            }
        }
        bool logJson = env_enabled("RECEIVER_LOG_TRAFFIC_JSON");
        bool logDetail = env_enabled("RECEIVER_LOG_TRAFFIC_DETAIL");
        bool logSummary = env_enabled("RECEIVER_LOG_TRAFFIC_SUMMARY") || logJson || logDetail;
        const bool trafficDumpEnabled = trafficDumpKeepSet && trafficDumpKeepMax >= 0;
        const bool lineLogEnabled = (getenv_int("TRAFFIC_PATH_LINE_LOG_ENABLE", 0) != 0);
        std::string trafficLineLogPath;
        if (lineLogEnabled) {
            trafficLineLogPath = getenv_str("TRAFFIC_PATH_LINE_LOG", "");
            if (trafficLineLogPath.empty() && !resultDumpDir.empty()) {
                trafficLineLogPath = (std::filesystem::path(resultDumpDir) / "traffic_path_line.log").string();
            }
        }
        trafficDebug.configure(resultDumpDir, trafficDumpKeepMax, logSummary, logJson, logDetail,
                               trafficDumpEnabled, trafficLineLogPath);
        if (trafficDumpEnabled) {
            if (!resultDumpDir.empty()) {
                if (startupSummary) {
                    std::cout << "ExternalReceiver: traffic JSON dump dir=" << resultDumpDir
                              << " keep=" << trafficDumpKeepMax << std::endl;
                }
            } else {
                if (startupSummary) {
                    std::cout << "ExternalReceiver: traffic JSON dump disabled (RESULT_DUMP_DIR empty)." << std::endl;
                }
            }
        }
        if (logSummary) {
            std::cout << "ExternalReceiver: traffic log enabled (stdout)"
                      << " json=" << (logJson ? "1" : "0")
                      << " detail=" << (logDetail ? "1" : "0") << std::endl;
        }
    }
    // 外部数据通道配置（可被环境变量覆盖）
    std::string exch = getenv_str("EXT_EXCHANGE", cfg.dispToAlgoExchange.c_str());
    std::string queue = getenv_str("EXT_QUEUE", cfg.dispToAlgoQueue.c_str());
    std::string bindingKey  = getenv_str("EXT_BINDING_KEY", cfg.dispToAlgoBindingKey.c_str());
    std::string exchType = getenv_str("EXT_EXCHANGE_TYPE", "fanout");
    double timeoutSec = getenv_double("EXT_TIMEOUT_SEC", 30.0);
    const bool purgeOnStart = env_enabled("PURGE_MQ_ON_START");
    int maxPriority = 0;
    if (const char* priorityEnv = std::getenv("EXT_QUEUE_MAX_PRIORITY")) {
        try {
            maxPriority = std::max(0, std::stoi(priorityEnv));
        } catch (...) {
            maxPriority = 0;
        }
    }
    int messageTtlMs = 0;
    if (const char* ttlEnv = std::getenv("EXT_MESSAGE_TTL_MS")) {
        try {
            messageTtlMs = std::max(0, std::stoi(ttlEnv));
        } catch (...) {
            messageTtlMs = 0;
        }
    }
    std::vector<std::string> purgeQueues;
    append_unique(purgeQueues, queue);
    append_unique(purgeQueues, getenv_str("ASSIGN_RESULT_QUEUE", cfg.algoToDispQueue.c_str()));
    append_unique(purgeQueues, getenv_str("TRAFFIC_PATH_QUEUE", cfg.algoToTrafficQueue.c_str()));
    // 可选：覆盖 AMR 的 max_time_ahead（秒）。<0 表示不覆盖，0 表示不限制
    int maxAheadOverrideSec = getenv_int("EXT_MAX_TIME_AHEAD_SEC", -1);
    const int resolvedAheadMs = (maxAheadOverrideSec >= 0) ? (maxAheadOverrideSec * 1000) : -1;
    int maxDetourOverrideMm = getenv_int("EXT_MAX_DETOUR_MM", -1); // 单位：mm，<0 表示不覆盖（保持默认0=无限制）
    const int resolvedDetourMm = (maxDetourOverrideMm >= 0) ? maxDetourOverrideMm : 0;
    if (startupSummary) {
        std::cout << "[Detour Params] maxAheadMs=" << resolvedAheadMs
                  << " maxDetourMm=" << resolvedDetourMm
                  << " (env: EXT_MAX_TIME_AHEAD_SEC / EXT_MAX_DETOUR_MM)" << std::endl;
    }

    int reconnectBaseMs = std::max(0, getenv_int("EXT_RECONNECT_BASE_MS", 1000));
    int reconnectMaxMs = std::max(reconnectBaseMs, getenv_int("EXT_RECONNECT_MAX_MS", 30000));
    auto reconnect_backoff_ms = [&](int attempt) -> int {
        int shift = std::min(attempt, 10);
        long long delay = static_cast<long long>(reconnectBaseMs) << shift;
        if (delay > reconnectMaxMs) delay = reconnectMaxMs;
        return static_cast<int>(delay);
    };
    auto cleanup_conn = [&](amqp_connection_state_t& conn, bool closeGracefully) {
        if (!conn) return;
        if (closeGracefully) {
            amqp_channel_close(conn, 1, AMQP_REPLY_SUCCESS);
            amqp_connection_close(conn, AMQP_REPLY_SUCCESS);
        }
        amqp_destroy_connection(conn);
        conn = nullptr;
    };

    bool purgedOnce = false;
    int reconnectAttempt = 0;
    while (true) {
        bool setupOk = true;
        bool connReady = false;
        amqp_connection_state_t conn = amqp_new_connection();
        amqp_socket_t* sock = amqp_tcp_socket_new(conn);
        if (!sock) {
            std::cerr << "ExternalReceiver: cannot create TCP socket" << std::endl;
            setupOk = false;
        }
        if (setupOk && amqp_socket_open(sock, cfg.host.c_str(), cfg.port)) {
            std::cerr << "ExternalReceiver: socket open failed" << std::endl;
            setupOk = false;
        }
        if (setupOk && !amqp_ok(amqp_login(conn, cfg.vhost.c_str(), 0, 131072, 0,
                                           AMQP_SASL_METHOD_PLAIN,
                                           cfg.username.c_str(), cfg.password.c_str()),
                                "login")) {
            setupOk = false;
        }
        if (setupOk) {
            amqp_channel_open(conn, 1);
            if (!amqp_ok(amqp_get_rpc_reply(conn), "channel.open")) setupOk = false;
        }

        // 声明资源（交换机类型可配置，默认与前端保持一致为 topic/fanout）
        if (setupOk) {
            amqp_exchange_declare(conn, 1,
                                  amqp_cstring_bytes(exch.c_str()),
                                  amqp_cstring_bytes(exchType.c_str()),
                                  0, 1, 0, 0, amqp_empty_table);
            if (!amqp_ok(amqp_get_rpc_reply(conn), "exchange.declare")) setupOk = false;
        }
        // 直接主动声明队列，与 sender 保持一致（durable=1，可选 x-max-priority / x-message-ttl），避免被动声明不存在队列导致 PRECONDITION_FAILED
        if (setupOk) {
            amqp_table_entry_t entries[2];
            amqp_table_t args{0, nullptr};
            int entryIdx = 0;
            if (maxPriority > 0) {
                entries[entryIdx].key = amqp_cstring_bytes("x-max-priority");
                entries[entryIdx].value.kind = AMQP_FIELD_KIND_U8;
                entries[entryIdx].value.value.u8 = static_cast<uint8_t>(std::min(maxPriority, 255));
                entryIdx++;
            }
            if (messageTtlMs > 0) {
                entries[entryIdx].key = amqp_cstring_bytes("x-message-ttl");
                entries[entryIdx].value.kind = AMQP_FIELD_KIND_I32;
                entries[entryIdx].value.value.i32 = messageTtlMs;
                entryIdx++;
            }
            if (entryIdx > 0) {
                args.entries = entries;
                args.num_entries = entryIdx;
            }
            amqp_queue_declare(conn, 1, amqp_cstring_bytes(queue.c_str()), 0, 1, 0, 0, args);
            if (!amqp_ok(amqp_get_rpc_reply(conn), "queue.declare")) setupOk = false;
        }
        if (setupOk) {
            amqp_queue_bind(conn, 1,
                            amqp_cstring_bytes(queue.c_str()),
                            amqp_cstring_bytes(exch.c_str()),
                            amqp_cstring_bytes(bindingKey.c_str()),
                            amqp_empty_table);
            if (!amqp_ok(amqp_get_rpc_reply(conn), "queue.bind")) setupOk = false;
        }
        if (setupOk && purgeOnStart && !purgedOnce) {
            purge_startup_queues(conn, purgeQueues);
            purgedOnce = true;
        }
        if (setupOk) {
            amqp_basic_consume(conn, 1, amqp_cstring_bytes(queue.c_str()), amqp_empty_bytes, 0, 1, 0, amqp_empty_table);
            if (!amqp_ok(amqp_get_rpc_reply(conn), "basic.consume")) setupOk = false;
        }

        if (!setupOk) {
            cleanup_conn(conn, false);
            int delayMs = reconnect_backoff_ms(reconnectAttempt++);
            std::cout << log_time_prefix() << "ExternalReceiver: reconnect in " << delayMs << " ms" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
            continue;
        }

        connReady = true;
        reconnectAttempt = 0;
        std::cout << log_time_prefix() << "ExternalReceiver: waiting on exchange='" << exch
                  << "' queue='" << queue << "' bindKey='" << bindingKey
                  << "' host=" << cfg.host << ":" << cfg.port << std::endl;

        // 简单循环接收消息（Ctrl+C 退出）
        bool inTimeout = false;
        while (true) {
            amqp_maybe_release_buffers(conn);
            struct timeval tv; tv.tv_sec = (int)timeoutSec; tv.tv_usec = (int)((timeoutSec - (int)timeoutSec)*1000000);
            amqp_envelope_t envm; amqp_rpc_reply_t rep = amqp_consume_message(conn, &envm, &tv, 0);
            if (rep.reply_type != AMQP_RESPONSE_NORMAL) {
                if (rep.reply_type == AMQP_RESPONSE_LIBRARY_EXCEPTION &&
                    rep.library_error == AMQP_STATUS_TIMEOUT) {
                    if (!inTimeout) {
                        std::cout << log_time_prefix() << "ExternalReceiver: timeout, continue..." << std::endl;
                        inTimeout = true;
                    }
                    continue;
                }
                amqp_ok(rep, "consume");
                break;
            }

            inTimeout = false;
            std::string body((char*)envm.message.body.bytes, envm.message.body.len);
            std::string rk((char*)envm.routing_key.bytes, envm.routing_key.len);
            const bool logEachMessage = env_enabled("RECEIVER_LOG_EACH_MESSAGE");
            const bool logEachMessageStatus = env_enabled("RECEIVER_LOG_EACH_MESSAGE_STATUS");
            const std::string statusRk = getenv_str("STATUS_ROUTING_KEY", "SendRobotStatusInfos");
            const bool rkIsStatus = (!statusRk.empty() && rk == statusRk);
            amqp_destroy_envelope(&envm);

            const bool rkIsStatusMessage = rkIsStatus;
            const bool rkKnown = rkIsStatusMessage || is_known_routing_key(rk);
            const bool isNoisy = rkIsStatusMessage || is_noisy_routing_key(rk);
            bool logHeader = rkKnown && log_enabled(isNoisy ? LogLevel::TRACE : LogLevel::INFO);
            if (!rkKnown) {
                std::cerr << log_time_prefix()
                          << "[Rx] unknown routing_key='" << rk << "' size=" << body.size()
                          << std::endl;
                dump_unknown_message(unknownRkDumpDir, body);
                if (log_enabled(LogLevel::WARN)) {
                    log_stream(LogLevel::WARN)
                        << log_time_prefix()
                        << "[Warn] Unknown routing_key='" << rk << "'"
                        << std::endl;
                }
            } else if (logHeader) {
                std::cout << log_time_prefix()
                          << "[Rx] routing_key='" << rk << "' size=" << body.size()
                          << std::endl;
            }
            if (logHeader) {
                std::cout << "\n" << log_time_prefix() << "--- Received message ---\n"
                          << log_time_prefix() << "size=" << body.size() << " bytes"
                          << ", routing_key='" << rk << "'\n";
            }

        // 解析 JSON 新格式：candidateTasks + 可选 agvStatusList
        json j;
            bool isStatusMessage = rkIsStatus;
            bool eachMessageLogged = false;
	        try {
	            j = json::parse(body);
                // 支持“单一 routingKey”模式：当业务侧把 status/path/trail 与 task 复用同一个 routingKey
                // (通常为 EXT_ROUTING_KEY，例如 AssignmentTaskRequest)，此处基于 payload 结构进行识别。
                bool looksLikeStatus = false;
                if (j.is_array()) {
                    // 绝大多数情况下，数组 payload 即为状态流（仿真端 send_status 默认发送数组）。
                    // 仅做轻量启发：首条看起来像 status 才处理，避免误把其他数组当成状态。
                    if (!j.empty()) {
                        const auto& first = j.front();
                        if (first.is_object()) {
                            bool hasId = first.contains("deviceId") || first.contains("agvId") || first.contains("agv_id");
                            bool hasPose = first.contains("nodeId") || first.contains("x") || first.contains("y");
                            if (hasId && hasPose) looksLikeStatus = true;
                        }
                    }
                }
                const bool hasRobotStatusInfos =
                    j.is_object()
                    && (j.contains("robotStatusInfos") || j.contains("RobotStatusInfos"));
                isStatusMessage = rkIsStatus || looksLikeStatus || hasRobotStatusInfos;

                const bool allowEachMessageLog =
                    logEachMessage
                    && (!isStatusMessage || logEachMessageStatus);
                if (allowEachMessageLog) {
                    if (!logHeader) {
                        std::cout << "\n" << log_time_prefix() << "--- Received message ---\n"
                                  << log_time_prefix() << "size=" << body.size() << " bytes"
                                  << ", routing_key='" << rk << "'\n";
                    }
                    eachMessageLogged = true;
                    if (j.is_object()) {
                        std::string msgId = read_string(j, "messageId", "");
                        std::string devId = read_string(j, "deviceId", "");
                        std::string subId = read_string(j, "subTaskId", "");
                        if (!msgId.empty() || !devId.empty() || !subId.empty()) {
                            std::cout << log_time_prefix() << "messageId=" << (msgId.empty() ? "<none>" : msgId)
                                      << " deviceId=" << (devId.empty() ? "<none>" : devId);
                            if (!subId.empty()) std::cout << " subTaskId=" << subId;
                            std::cout << std::endl;
                        }
                    } else if (j.is_array()) {
                        std::cout << log_time_prefix() << "json=array len=" << j.size() << std::endl;
                    }
                }

                if (looksLikeStatus) {
                    handle_status_infos_multimap(j, mapManager);
                    continue;
                }
            if (rkIsStatus) {
                handle_status_infos_multimap(j, mapManager);
                continue;
            }
            if (rk == "SendRobotConfigInfos") {
                handle_config_infos_multimap(j, mapManager);
                continue;
            }
            if (j.contains("robotStatusInfos")) {
                handle_status_infos_multimap(j, mapManager);
            }
            if (rk == "SendMapInfo" || j.contains("mapData")) {
                int incomingMapId = detect_map_id_from_map_message(j);
                auto ctx = mapManager.getOrCreateContext(incomingMapId);
                json mapPayload = j;
                mapReloadQueue.submit([&, ctx, mapPayload]() {
                    if (reload_context_map_payload(mapPayload,
                                                   *ctx,
                                                   getenv_str("MAP_CACHE_FILE", "debug/received_map.json"),
                                                   startupSummary)) {
                        if (!initSentAfterMap.exchange(true)) {
                            send_initialize();
                        }
                        initTickerRunning.store(false);
                        std::cout << log_time_prefix()
                                  << "[Map] map info reload success mapId="
                                  << ctx->activeMapId.load() << "\n";
                    }
                });
                if (rk == "SendMapInfo") {
                    continue;
                }
            }
            const std::string extUnifiedRk = getenv_str("EXT_ROUTING_KEY", "");
            const bool rkIsUnified = (!extUnifiedRk.empty() && rk == extUnifiedRk);
            const bool maybeRouteReqPayload =
                j.is_object()
                && j.contains("messageId")
                && j.contains("deviceId")
                && !j.contains("candidateTasks")
                && !j.contains("mapData")
                && !j.contains("robotStatusInfos")
                && !j.contains("RobotStatusInfos")
                && !j.contains("robotConfigInfos")
                && !j.contains("RobotConfigInfos");
            const bool allowUnifiedRouteReq = rkIsUnified && maybeRouteReqPayload;

            if (rk == "RobotAroundPathRequest") {
                log_line(LogLevel::INFO, "[AroundPathRequest] payload=" + j.dump());
                std::string messageId = read_string(j, "messageId", "");
                std::string deviceId = read_string(j, "deviceId", "");
                std::string subTaskId = read_string(j, "subTaskId", "");
                int requestMapId = read_request_map_id(j, 0);
                auto ctx = mapManager.resolveContextForDevice(deviceId, requestMapId);

                auto fail_resp = [&](int code, const std::string& err) {
                    json resp;
                    resp["messageId"] = messageId;
                    resp["deviceId"] = deviceId;
                    resp["code"] = code;
                    resp["error"] = err;
                    resp["pathInfos"] = json::array();
                    algoPublisher.sendAroundPathResponse(resp);
                };

                if (messageId.empty()) {
                    fail_resp(400, "RobotAroundPathRequest missing messageId");
                    continue;
                }
                if (deviceId.empty()) {
                    fail_resp(400, "RobotAroundPathRequest missing deviceId");
                    continue;
                }
                if (!ctx) {
                    fail_resp(404, "No active map context for device");
                    continue;
                }
                if (subTaskId.empty()) {
                    fail_resp(400, "RobotAroundPathRequest missing subTaskId");
                    continue;
                }
                if (!j.contains("x") || !j.contains("y") || !j.contains("angle")) {
                    fail_resp(400, "RobotAroundPathRequest missing x/y/angle");
                    continue;
                }

                int x = read_int(j, "x", 0);
                int y = read_int(j, "y", 0);
                (void)read_int(j, "angle", 0);

                std::vector<ObstaclePointMm> obstacles;
                if (j.contains("obstacles") && j["obstacles"].is_array()) {
                    for (const auto& ob : j["obstacles"]) {
                        if (!ob.is_object()) continue;
                        ObstaclePointMm p;
                        p.x = read_double(ob, "x", 0.0);
                        p.y = read_double(ob, "y", 0.0);
                        obstacles.push_back(p);
                    }
                }

                int waitMs = getenv_int("PATH_REQ_WAIT_MS", 2000);
                auto planOpt = wait_for_plan(ctx->robotRepo, deviceId, waitMs);
                if (!planOpt || deviceId.empty()) {
                    fail_resp(404, "No cached plan for device");
                    continue;
                }
                PlanCacheEntry basePlan = *planOpt;
                if (basePlan.plan.segments.empty()) {
                    fail_resp(404, "No cached plan segments for device");
                    continue;
                }

                std::shared_lock<std::shared_mutex> mapReadLock(ctx->mapMutex);
                if (!ctx->mapReady.load() || !ctx->aStarPtr) {
                    fail_resp(503, "Map not ready");
                    continue;
                }

                int currentNodeId = -1;
                try {
                    int refNodeId = -1;
                    auto stOpt = ctx->robotRepo.getStatusById(deviceId);
                    if (stOpt.has_value()) {
                        refNodeId = resolve_status_node_id(*stOpt, *ctx->mapInfoPtr);
                    }
                    if (refNodeId >= 0) {
                        currentNodeId = ctx->mapInfoPtr->findNearestNodeIdFromNeighbors(x, y, refNodeId);
                    } else {
                        currentNodeId = ctx->mapInfoPtr->findNearestNodeId(x, y);
                    }
                } catch (...) {
                    currentNodeId = -1;
                }
                if (currentNodeId < 0) {
                    fail_resp(422, "Cannot map (x,y) to graph node");
                    continue;
                }

                int segStartIdx = -1;
                for (size_t i = 0; i < basePlan.plan.segments.size(); ++i) {
                    const auto& seg = basePlan.plan.segments[i];
                    const std::string segSubTaskId = resolve_segment_subtask_id(seg);
                    if (!segSubTaskId.empty() && segSubTaskId == subTaskId) {
                        segStartIdx = static_cast<int>(i);
                        break;
                    }
                }
                if (segStartIdx < 0) {
                    fail_resp(422, "subTaskId not found in cached plan");
                    continue;
                }

                double nodeBlockMm = getenv_double("AROUND_PATH_NODE_BLOCK_MM", 300.0);
                double edgeBlockMm = getenv_double("AROUND_PATH_EDGE_BLOCK_MM", 300.0);
                nodeBlockMm = std::max(0.0, nodeBlockMm);
                edgeBlockMm = std::max(0.0, edgeBlockMm);
                BlockedGraph blocked = compute_blocked_graph_from_obstacles(*ctx->mapInfoPtr, obstacles, nodeBlockMm, edgeBlockMm);

                PathPlanningHelper::AmrPlanInfo newPlan;
                newPlan.amrId = deviceId;
                newPlan.startNodeId = currentNodeId;
                newPlan.totalDistanceMm = 0.0;
                newPlan.totalTimeSec = 0.0;
                newPlan.allReachable = true;

                int fromNodeId = currentNodeId;
                for (size_t i = static_cast<size_t>(segStartIdx); i < basePlan.plan.segments.size(); ++i) {
                    const auto& oldSeg = basePlan.plan.segments[i];
                    const int goalNodeId = oldSeg.toNodeId;
                    if (goalNodeId < 0) {
                        fail_resp(500, "Cached plan has invalid target node");
                        newPlan.segments.clear();
                        break;
                    }
                    if (blocked.blockedNodes.count(goalNodeId)) {
                        fail_resp(500, "Target node blocked by obstacle: " + std::to_string(goalNodeId));
                        newPlan.segments.clear();
                        break;
                    }

                    std::vector<int> route;
                    bool usedStatic = false;
                    if (!ctx->skipStaticTable && ctx->staticTableReady) {
                        StaticPathTable::DynamicContext dynCtx;
                        dynCtx.blockedNodes = blocked.blockedNodes;
                        dynCtx.blockedEdges = blocked.blockedEdges;
                        dynCtx.blockedNodes.erase(fromNodeId);
                        auto cand = ctx->staticTable.query(fromNodeId, goalNodeId, dynCtx);
                        if (cand.has_value() && !cand->fullPath.empty()) {
                            route = cand->fullPath;
                            usedStatic = true;
                        }
                    }
                    if (route.empty()) {
                        std::unordered_set<int> bannedNodes(blocked.blockedNodes.begin(), blocked.blockedNodes.end());
                        bannedNodes.erase(fromNodeId);
                        const std::unordered_set<int>* bannedPtr = bannedNodes.empty() ? nullptr : &bannedNodes;
                        auto res = ctx->aStarPtr->findPath(
                            fromNodeId,
                            goalNodeId,
                            PathPlanningConstants::resolveTurnPenaltyMm(),
                            bannedPtr,
                            &blocked.blockedEdges);
                        if (res.found && !res.path.empty()) {
                            route = res.path;
                        }
                    }

                    if (route.empty()) {
                        fail_resp(500, "Unreachable due to blocked node/edge");
                        newPlan.segments.clear();
                        break;
                    }
                    if (route.front() != fromNodeId) {
                        route.insert(route.begin(), fromNodeId);
                    }

                    PathPlanningHelper::PathSegmentInfo seg = oldSeg;
                    seg.fromNodeId = fromNodeId;
                    seg.toNodeId = goalNodeId;
                    seg.nodes = std::move(route);
                    seg.reachable = true;
                    seg.source = usedStatic ? GlobalPathPlanner::PathSource::STATIC_TABLE : GlobalPathPlanner::PathSource::PURE_ASTAR;
                    seg.distanceMm = compute_route_distance_mm(*ctx->mapInfoPtr, seg.nodes);
                    double speed = ctx->mapInfoPtr->getGlobalMaxSpeed();
                    if (!(speed > 0.0)) speed = 1000.0;
                    seg.timeSec = seg.distanceMm / speed;
                    seg.description = "around_replan (" + std::to_string(seg.fromNodeId) + "->" + std::to_string(seg.toNodeId) + ")";

                    newPlan.totalDistanceMm += seg.distanceMm;
                    newPlan.totalTimeSec += seg.timeSec;
                    newPlan.totalTimeSec += seg.serviceTimeSec;
                    newPlan.segments.push_back(std::move(seg));
                    fromNodeId = goalNodeId;
                }

                if (newPlan.segments.empty()) {
                    continue;
                }

                std::string schedId = basePlan.schedulingRequestId.empty() ? messageId : basePlan.schedulingRequestId;
                std::string genTime = iso8601_utc_now();
                ctx->robotRepo.updatePlan(deviceId, newPlan, schedId, genTime, basePlan.taskPriorities);

                {
                    auto updatedPlanOpt = ctx->robotRepo.getPlan(deviceId);
                    if (updatedPlanOpt.has_value()) {
                        int committedNextNode = -1;
                        int batteryLevel = 100;
                        int deviceType = 0;
                        auto stOpt = ctx->robotRepo.getStatusById(deviceId);
                        if (stOpt.has_value()) {
                            committedNextNode = resolve_committed_next_node_id(*stOpt, *ctx->mapInfoPtr);
                            batteryLevel = stOpt->batteryLevel;
                            deviceType = stOpt->deviceType;
                        }
                        const int reserveBudgetOverride = 0;
                        compute_dynamic_plan_to_next_target(
                            deviceId,
                            currentNodeId,
                            committedNextNode,
                            batteryLevel,
                            deviceType,
                            *updatedPlanOpt,
                            ctx->robotRepo,
                            *ctx->mapInfoPtr,
                            ctx->skipStaticTable,
                            ctx->staticTableReady,
                            ctx->staticTable,
                            *ctx->aStarPtr,
                            ctx->nodeReservations,
                            reserveBudgetOverride,
                            true,
                            ReplanStage::NONE,
                            nullptr);
                    }
                }

                PlanCacheEntry respEntry;
                respEntry.plan = newPlan;
                auto respOpt = build_path_response_body(messageId, deviceId, respEntry, *ctx->mapInfoPtr);
                if (respOpt.has_value()) {
                    algoPublisher.sendAroundPathResponse(*respOpt);
                }

                std::unordered_map<std::string, int> emptyPriorities;
                ordered_json trafficPayload =
                    build_traffic_path_payload(messageId, ctx->activeMapId.load(), ctx->robotRepo, *ctx->mapInfoPtr, emptyPriorities,
                                               ctx->mapReady.load(), ctx->staticTableReady, ctx->skipStaticTable, ctx->staticTable,
                                               ctx->aStarPtr.get(), ctx->nodeReservations);
                attach_map_metadata(trafficPayload, *ctx);
                std::string trafficPayloadStr = trafficPayload.dump();
                trafficPathPublisher.publishPayload(trafficPayloadStr);
                trafficDebug.onPublish(trafficPayload, trafficPayloadStr, "replan");
                {
                    const bool logSummary = env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL");
                    const bool logDetail = env_enabled("RECEIVER_LOG_ROUTE_DETAIL");
                    auto planOpt = ctx->robotRepo.getPlan(deviceId);
                    auto trailRespOpt = build_trail_response_for_device(
                        messageId,
                        deviceId,
                        subTaskId,
                        planOpt,
                        ctx->robotRepo,
                        *ctx->mapInfoPtr,
                        ctx->mapReady.load(),
                        ctx->skipStaticTable,
                        ctx->staticTableReady,
                        ctx->staticTable,
                        ctx->aStarPtr.get(),
                        ctx->nodeReservations,
                        logSummary,
                        logDetail);
                    if (trailRespOpt.has_value()) {
                        algoPublisher.sendTrailResponse(*trailRespOpt);
                    }
                }
                continue;
            }

            if (rk == "RobotPathRequest" || allowUnifiedRouteReq) {
                std::string messageId = read_string(j, "messageId", "");
                std::string deviceId = read_string(j, "deviceId", "");
                std::string subTaskId = read_string(j, "subTaskId", "");
                int requestMapId = read_request_map_id(j, 0);
                auto ctx = mapManager.resolveContextForDevice(deviceId, requestMapId);
                auto contains_nocase = [](const std::string& s, const std::string& needle) {
                    auto it = std::search(
                        s.begin(), s.end(),
                        needle.begin(), needle.end(),
                        [](char a, char b) { return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b)); });
                    return it != s.end();
                };
                const bool looksLikeTrail = (!subTaskId.empty())
                    || contains_nocase(messageId, "trail");
                const bool looksLikePath = contains_nocase(messageId, "path");
                if (allowUnifiedRouteReq && looksLikeTrail && !looksLikePath) {
                    // 统一 routingKey 下的 trail 请求：交由 trail handler 处理（避免误回 path response）。
                } else {
                int waitMs = getenv_int("PATH_REQ_WAIT_MS", 2000);
                auto planOpt = ctx ? wait_for_plan(ctx->robotRepo, deviceId, waitMs) : std::optional<PlanCacheEntry>{};
                std::optional<json> respOpt;
                if (ctx) {
                    std::shared_lock<std::shared_mutex> mapReadLock(ctx->mapMutex);
                    if (planOpt && !deviceId.empty()) {
                        PlanCacheEntry planned = *planOpt;
                        int currentNode = planned.plan.startNodeId;
                        auto stOpt = ctx->robotRepo.getStatusById(deviceId);
                        if (stOpt.has_value()) {
                            currentNode = resolve_status_node_id(*stOpt, *ctx->mapInfoPtr);
                        }
                        if (planned.plan.segments.empty() && currentNode >= 0) {
                            PathPlanningHelper::AmrPlanInfo idle;
                            idle.amrId = deviceId;
                            idle.startNodeId = currentNode;
                            PathPlanningHelper::PathSegmentInfo seg;
                            seg.fromNodeId = currentNode;
                            seg.toNodeId = currentNode;
                            seg.nodes = {currentNode};
                            seg.stepType = "-1";
                            seg.reachable = true;
                            idle.segments.push_back(std::move(seg));
                            planned.plan = std::move(idle);
                        }
                        respOpt = build_path_response_body(messageId, deviceId, planned, *ctx->mapInfoPtr);
                    } else {
                        json resp;
                        resp["messageId"] = messageId;
                        resp["deviceId"] = deviceId;
                        resp["code"] = 404;
                        resp["error"] = "No cached plan for device";
                        resp["pathInfos"] = json::array();
                        respOpt = std::move(resp);
                    }
                } else {
                    json resp;
                    resp["messageId"] = messageId;
                    resp["deviceId"] = deviceId;
                    resp["code"] = 404;
                    resp["error"] = "No active map context for device";
                    resp["pathInfos"] = json::array();
                    respOpt = std::move(resp);
                }
                if (!respOpt.has_value()) {
                    continue;
                }
                if (messageId.empty()) {
                    std::cout << log_time_prefix() << "[Warn] RobotPathRequest 缺少 messageId，响应无法对齐，请求方应填充。" << std::endl;
                } else if (env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL")) {
                    std::cout << log_time_prefix() << "[PathResp] echo messageId=" << messageId << " deviceId=" << deviceId << std::endl;
                }
                algoPublisher.sendPathResponse(*respOpt);
                continue;
                }
            }
	            if (rk == "RobotTrailRequest" || allowUnifiedRouteReq) {
	                std::string messageId = read_string(j, "messageId", "");
	                std::string deviceId = read_string(j, "deviceId", "");
	                std::string subTaskId = read_string(j, "subTaskId", "");
                int requestMapId = read_request_map_id(j, 0);
                auto ctx = mapManager.resolveContextForDevice(deviceId, requestMapId);
                if (allowUnifiedRouteReq) {
                    auto contains_nocase = [](const std::string& s, const std::string& needle) {
                        auto it = std::search(
                            s.begin(), s.end(),
                            needle.begin(), needle.end(),
                            [](char a, char b) { return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b)); });
                        return it != s.end();
                    };
                    const bool looksLikePath = contains_nocase(messageId, "path");
                    const bool looksLikeTrail = (!subTaskId.empty())
                        || contains_nocase(messageId, "trail");
                    if (looksLikePath && !looksLikeTrail) {
	                        // 统一 routingKey 下的 path 请求：由 path handler 处理。
	                        continue;
	                    }
	                }
	                const bool logSummary = env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL");
	                const bool logDetail = env_enabled("RECEIVER_LOG_ROUTE_DETAIL");
	                int waitMs = getenv_int("PATH_REQ_WAIT_MS", 2000);
                    if (subTaskId.empty()) {
                        std::cout << log_time_prefix() << "[TrailReq] skip empty subTaskId messageId="
                                  << (messageId.empty() ? "<none>" : messageId)
                                  << " deviceId=" << (deviceId.empty() ? "<none>" : deviceId)
                                  << std::endl;
                        continue;
                    }
	                if (logSummary) {
	                    std::cout << log_time_prefix() << "[TrailReq] messageId=" << (messageId.empty() ? "<none>" : messageId)
	                              << " deviceId=" << (deviceId.empty() ? "<none>" : deviceId)
	                              << " subTaskId=" << (subTaskId.empty() ? "<none>" : subTaskId)
	                              << " waitMs=" << waitMs
	                              << std::endl;
	                }
                    std::optional<PlanCacheEntry> planOpt;
                    json resp;
                    if (ctx) {
                        planOpt = wait_for_plan(ctx->robotRepo, deviceId, waitMs);
                        std::shared_lock<std::shared_mutex> mapReadLock(ctx->mapMutex);
                        auto respOpt = build_trail_response_for_device(
                            messageId,
                            deviceId,
                            subTaskId,
                            planOpt,
                            ctx->robotRepo,
                            *ctx->mapInfoPtr,
                            ctx->mapReady.load(),
                            ctx->skipStaticTable,
                            ctx->staticTableReady,
                            ctx->staticTable,
                            ctx->aStarPtr.get(),
                            ctx->nodeReservations,
                            logSummary,
                            logDetail);
                        if (!respOpt.has_value()) {
                            continue;
                        }
                        resp = std::move(*respOpt);
                    } else {
                        resp["messageId"] = messageId;
                        resp["deviceId"] = deviceId;
                        resp["code"] = 404;
                        resp["error"] = "No active map context for device";
                        resp["controlPoints"] = json::array();
                    }
                if (messageId.empty()) {
                    std::cout << log_time_prefix() << "[Warn] RobotTrailRequest 缺少 messageId，响应无法对齐，请求方应填充。" << std::endl;
                } else if (env_enabled("RECEIVER_LOG_ROUTE_SUMMARY") || env_enabled("RECEIVER_LOG_ROUTE_DETAIL")) {
                    std::cout << log_time_prefix() << "[TrailResp] echo messageId=" << messageId << " deviceId=" << deviceId << std::endl;
                }
                algoPublisher.sendTrailResponse(resp);
                continue;
            }
            // 新格式：candidateTasks（批量调度请求）
            if (j.contains("candidateTasks") && j["candidateTasks"].is_array()) {
                auto splitPayloads = split_scheduling_payloads_by_map(j, mapManager);
                if (splitPayloads.empty()) {
                    publish_rejected_allocation(resultPublisher, j, "no_resolved_map_context", 404);
                    continue;
                }
                for (auto& item : splitPayloads) {
                    const int targetMapId = item.first;
                    json requestPayload = item.second;
                    json payloadForMap = requestPayload;
                    auto ctx = mapManager.getOrCreateContext(targetMapId);
                    std::optional<json> dropped;
                    bool rejectBusy = false;
                    bool shouldStartWorker = false;
                    {
                        std::lock_guard<std::mutex> lk(ctx->schedulingLatestMutex);
                        if (ctx->schedulingBusy.load(std::memory_order_relaxed)) {
                            rejectBusy = true;
                        } else {
                            if (ctx->schedulingLatest.has_value()) {
                                dropped = std::move(ctx->schedulingLatest);
                            }
                            ctx->schedulingLatest = std::move(payloadForMap);
                            if (!ctx->schedulingWorkerActive) {
                                ctx->schedulingWorkerActive = true;
                                shouldStartWorker = true;
                            }
                        }
                    }
                    if (shouldStartWorker) {
                        schedulingQueue.submit([&, ctx]() {
                            while (true) {
                                std::optional<json> payload;
                                {
                                    std::lock_guard<std::mutex> lk(ctx->schedulingLatestMutex);
                                    if (!ctx->schedulingLatest.has_value()) {
                                        ctx->schedulingWorkerActive = false;
                                        break;
                                    }
                                    payload = std::move(ctx->schedulingLatest);
                                    ctx->schedulingLatest.reset();
                                }
                                ctx->schedulingBusy.store(true, std::memory_order_relaxed);
                                SchedulingBusyGuard busyGuard(ctx->schedulingBusy);
                                SchedulingBusyCountGuard busyCountGuard(schedulingBusyCount);
                                try {
                                    processSchedulingMessage(std::move(*payload),
                                        ctx->robotRepo, ctx->nodeReservations, *ctx->mapInfoPtr, ctx->mapMutex,
                                        ctx->mapReady, ctx->activeMapId, ctx->aStarPtr,
                                        ctx->staticTable, ctx->staticTableReady, ctx->skipStaticTable,
                                        resultPublisher, algoPublisher, trafficPathPublisher, trafficDebug, serviceName,
                                        resolvedAheadMs, resolvedDetourMm);
                                } catch (const std::exception& ex) {
                                    std::cerr << "[SchedulingWorker] job exception: " << ex.what() << std::endl;
                                } catch (...) {
                                    std::cerr << "[SchedulingWorker] job unknown exception" << std::endl;
                                }
                            }
                        });
                    }
                    if (rejectBusy) {
                        if (env_enabled("RECEIVER_LOG_REJECT_RESP")) {
                            std::cout << "[Warn] 调度忙碌中，当前请求直接返回404 mapId=" << targetMapId << std::endl;
                        }
                        publish_rejected_allocation(resultPublisher, requestPayload, "busy_request", 404);
                        continue;
                    }
                    if (dropped.has_value()) {
                        if (env_enabled("RECEIVER_LOG_REJECT_RESP")) {
                            std::cout << "[Warn] 调度请求被最新请求替换，直接返回404 mapId=" << targetMapId << std::endl;
                        }
                        publish_rejected_allocation(resultPublisher, *dropped, "stale_request", 404);
                    }
                }
                continue;
            }

        } catch (const std::exception& e) {
            if (logEachMessage && (!isStatusMessage || logEachMessageStatus) && !eachMessageLogged) {
                std::cout << "\n" << log_time_prefix() << "--- Received message ---\n"
                          << log_time_prefix() << "size=" << body.size() << " bytes"
                          << ", routing_key='" << rk << "'\n";
            }
            std::cerr << "ExternalReceiver: parse failed: " << e.what() << std::endl;
        }
        }

        cleanup_conn(conn, connReady);
        int delayMs = reconnect_backoff_ms(reconnectAttempt++);
        std::cout << log_time_prefix() << "ExternalReceiver: reconnect in " << delayMs << " ms" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
    }

    schedulingQueue.stop();
    // 不会到达此处
    heartbeatRunning = false;
    if (heartbeatThread.joinable()) heartbeatThread.join();
    if (simPoseStreamer) simPoseStreamer->stop();
    return 0;
}
