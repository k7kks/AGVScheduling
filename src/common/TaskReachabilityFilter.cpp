#include "common/TaskReachabilityFilter.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <unordered_set>
#include <thread>
#include <cstdlib>
#include <string>
#include <memory>
#include "algorithm/base/TaskAllocationUtils.h"
#include "algorithm/ShortestPathUpdater.h"
#include "common/AmrPositionResolver.h"
#include "common/PathPlanningConstants.h"

#include <unordered_map>

namespace {

int ReadEnvInt(const char* key, int defv) {
    if (const char* v = std::getenv(key)) {
        try { return std::stoi(v); } catch (...) { return defv; }
    }
    return defv;
}

bool ReadEnvBool(const char* key, bool defv) {
    if (const char* v = std::getenv(key)) {
        std::string s(v);
        if (s == "0" || s == "false" || s == "False" || s == "FALSE") return false;
        return true;
    }
    return defv;
}

double TravelSeconds(int fromNode, int toNode, int amrType,
                     ShortestPathUpdater& pathUpdater,
                     const MapInfo& mapInfo);
double StraightLineSec(int fromNode, int toNode, const MapInfo& mapInfo);

double TravelSecondsWithWait(int fromNode, int toNode, int amrType,
                             ShortestPathUpdater& pathUpdater,
                             const MapInfo& mapInfo,
                             int deadline_ms = 300) {  // 超过上限视为不可达
    if (fromNode < 0 || toNode < 0) return std::numeric_limits<double>::infinity();
    try {
        std::vector<int> targets{toNode};
        pathUpdater.batchQueryByNodeId(fromNode, targets, amrType);
        auto tBeg = std::chrono::steady_clock::now();
        const int step = 10;
        while (true) {
            double sec = pathUpdater.queryByNodeId(fromNode, toNode, amrType);
            if (sec == static_cast<double>(DistanceState::UNREACHABLE)) {
                return std::numeric_limits<double>::infinity();
            }
            if (sec > 0.0) {
                return sec;
            }
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(now - tBeg).count();
            if (elapsed >= deadline_ms) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(step));
        }
        // 超过上限仍未得到有效结果，视为不可达
        return std::numeric_limits<double>::infinity();
    } catch (...) {
        return std::numeric_limits<double>::infinity();
    }
}

double TravelSecondsGated(int fromNode, int toNode, int amrType,
                          ShortestPathUpdater& pathUpdater,
                          const MapInfo& mapInfo,
                          bool useSpu,
                          int deadline_ms = 300) {
    if (fromNode < 0 || toNode < 0) return std::numeric_limits<double>::infinity();
    if (!mapInfo.isConnectedById(fromNode, toNode)) {
        return std::numeric_limits<double>::infinity();
    }
    if (!useSpu) {
        return StraightLineSec(fromNode, toNode, mapInfo);
    }
    return TravelSecondsWithWait(fromNode, toNode, amrType, pathUpdater, mapInfo, deadline_ms);
}

std::vector<int> CollectTaskNodes(const Task& task) {
    std::vector<int> nodes;
    const auto& subs = task.getSubTasks();
    nodes.reserve(subs.size());
    for (const auto& sub : subs) {
        int nid = sub.getPoint().getNodeId();
        if (nid >= 0) nodes.push_back(nid);
    }
    if (nodes.empty()) {
        int start = task.getStartId();
        int end = task.getEndId();
        if (start >= 0) nodes.push_back(start);
        if (end >= 0 && end != start) {
            nodes.push_back(end);
        }
    }
    return nodes;
}

bool IsTaskReachable(const Task& task,
	                const MapInfo& mapInfo,
	                    const std::vector<int>& amrTypes,
	                    const std::function<double(int,int,int)>& query_fn,
	                    bool useSpu,
	                    int& failedFrom,
                    int& failedTo,
                    std::string& reason) {
    const auto& id2index = mapInfo.getId2Index();
    auto nodes = CollectTaskNodes(task);
    if (nodes.empty()) {
        failedFrom = failedTo = -1;
        reason = "任务缺少任何路径节点";
        return false;
    }
    if (nodes.size() == 1) {
        if (nodes.front() < 0) {
            failedFrom = failedTo = -1;
            reason = "任务首节点无效";
            return false;
        }
        // 单节点任务视为可达（无需移动）
        return true;
    }
    for (size_t i = 0; i + 1 < nodes.size(); ++i) {
        int from = nodes[i];
        int to = nodes[i + 1];
        if (from < 0 || to < 0) {
            failedFrom = from;
            failedTo = to;
            reason = "节点ID缺失或非法";
            return false;
        }
        if (id2index.find(from) == id2index.end()) {
            failedFrom = from;
            failedTo = to;
            reason = "起点节点不存在于地图";
            return false;
        }
        if (id2index.find(to) == id2index.end()) {
            failedFrom = from;
            failedTo = to;
            reason = "终点节点不存在于地图";
            return false;
        }
        if (!mapInfo.isConnectedById(from, to)) {
            failedFrom = from;
            failedTo = to;
            reason = "起点终点不连通（reachable_analyzer 不可达）";
            return false;
        }
        bool reachable = !useSpu;
        if (useSpu) {
            for (int type : amrTypes) {
                double seg = query_fn(from, to, type);
                if (std::isfinite(seg)) { reachable = true; break; }
            }
        }
        if (!reachable) {
            failedFrom = from;
            failedTo = to;
            reason = "SPU 判定不可达";
            return false;
        }
    }
    return true;
}

struct TaskEndpoints {
    int start;
    int end;
};

TaskEndpoints ResolveEndpoints(const Task& task) {
    TaskEndpoints ep{-1, -1};
    const auto& subs = task.getSubTasks();
    if (!subs.empty()) {
        for (const auto& st : subs) {
            int nid = st.getPoint().getNodeId();
            if (nid >= 0) { ep.start = nid; break; }
        }
        for (auto it = subs.rbegin(); it != subs.rend(); ++it) {
            int nid = it->getPoint().getNodeId();
            if (nid >= 0) { ep.end = nid; break; }
        }
    } else {
        ep.start = task.getStartId();
        ep.end = task.getEndId();
    }
    if (ep.end < 0) ep.end = ep.start;
    return ep;
}

double CalcServiceSec(const Task& task) {
    double svc = 0.0;
    for (const auto& sub : task.getSubTasks()) {
        svc += std::max<std::int64_t>(0, sub.getEstimatedDurationMs());
    }
    return svc / 1000.0;
}

double StraightLineSec(int fromNode, int toNode, const MapInfo& mapInfo) {
    if (fromNode < 0 || toNode < 0) return std::numeric_limits<double>::infinity();
    try {
        const Node& nf = mapInfo.getNodeById(fromNode);
        const Node& nt = mapInfo.getNodeById(toNode);
        double dx = nf.x - nt.x;
        double dy = nf.y - nt.y;
        double dist = std::abs(dx) + std::abs(dy);  // mm
        double speed = mapInfo.getGlobalMaxSpeed();
        if (!(speed > 0)) speed = 1000.0;
        return dist / speed;
    } catch (...) {
        return std::numeric_limits<double>::infinity();
    }
}

double TravelSeconds(int fromNode, int toNode, int amrType,
                     ShortestPathUpdater& pathUpdater,
                     const MapInfo& mapInfo) {
    if (fromNode < 0 || toNode < 0) {
        return std::numeric_limits<double>::infinity();
    }
    double sec = 0.0;
    try {
        sec = pathUpdater.queryByNodeId(fromNode, toNode, amrType);
    } catch (...) {
        sec = static_cast<double>(DistanceState::UNCALCULATED);
    }
    if (sec == static_cast<double>(DistanceState::UNREACHABLE)) {
        return std::numeric_limits<double>::infinity();
    }
    if (sec == static_cast<double>(DistanceState::UNCALCULATED) || !(sec > 0.0)) {
        return StraightLineSec(fromNode, toNode, mapInfo);
    }
    return sec;
}

double TaskInternalTravelSec(const Task& task,
                             int amrType,
                             ShortestPathUpdater& pathUpdater,
                             const MapInfo& mapInfo,
                             bool useSpu) {
    auto nodes = CollectTaskNodes(task);
    if (nodes.size() <= 1) return 0.0;
    double total = 0.0;
    for (size_t i = 0; i + 1 < nodes.size(); ++i) {
        double seg = TravelSecondsGated(nodes[i], nodes[i + 1], amrType, pathUpdater, mapInfo, useSpu, 300);
        if (!std::isfinite(seg)) return seg;
        total += seg;
    }
    return total;
}

std::string NormalizeStartIso(const Task& task) {
    if (!task.getExpectedStartTimeISO().empty()) {
        return task.getExpectedStartTimeISO();
    }
    if (!task.getCreateTimestampISO().empty()) {
        return task.getCreateTimestampISO();
    }
    return std::string("9999-12-31T23:59:59Z");
}

double ChainDurationSec(const std::vector<int>& chain,
                        const Amr& amr,
                        const std::vector<Task>& tasks,
                        ShortestPathUpdater& pathUpdater,
                        const MapInfo& mapInfo,
                        bool useSpu) {
    if (chain.empty()) return 0.0;
    double total = 0.0;
    int currentNode = AmrPositionResolver::resolveStartNodeId(amr, mapInfo);
    int amrType = amr.getDeviceType();
    for (int taskIdx : chain) {
        if (taskIdx < 0 || taskIdx >= static_cast<int>(tasks.size())) {
            return std::numeric_limits<double>::infinity();
        }
        const Task& task = tasks[taskIdx];
        auto nodes = CollectTaskNodes(task);
        if (!nodes.empty()) {
            double entry = TravelSecondsGated(currentNode, nodes.front(), amrType, pathUpdater, mapInfo, useSpu, 300);
            if (!std::isfinite(entry)) return entry;
            total += entry;
            double internal = TaskInternalTravelSec(task, amrType, pathUpdater, mapInfo, useSpu);
            if (!std::isfinite(internal)) return internal;
            total += internal;
            currentNode = nodes.back();
        }
        total += CalcServiceSec(task);
    }
    return total;
}

}  // namespace

namespace TaskReachabilityFilter {

FilterResult FilterTasksByReachability(const std::vector<Task>& tasks,
                                       const MapInfo& mapInfo,
                                       const std::vector<Amr>& candidateAmrs) {
    FilterResult result;
    result.filteredTasks.reserve(tasks.size());
    result.filteredToOriginal.reserve(tasks.size());
    result.originalToFiltered.assign(tasks.size(), -1);
    result.forcedReasons.assign(tasks.size(), "");

    // 默认只做地图连通性快速筛（避免每轮创建 SPU 线程池导致卡死/抖动）。
    // 如需更严格的可达性检查，可显式设置 REACH_START_SPU=1。
    const bool startSpuEnabled = ReadEnvBool("REACH_START_SPU", false);
    std::unique_ptr<ShortestPathUpdater> pathUpdater;
    ShortestPathUpdater* pathUpdaterPtr = nullptr;
    if (startSpuEnabled) {
        int workerNum = ReadEnvInt("REACH_WORKERS", 4);
        workerNum = std::max(1, std::min(workerNum, 16));
        pathUpdater.reset(new ShortestPathUpdater(
            mapInfo,
            PathPlanningConstants::kDefaultTurnPenaltyMm,
            workerNum
        ));
        pathUpdaterPtr = pathUpdater.get();
    }
    int maxTypeIdx = 0;
    for (const auto& a : candidateAmrs) {
        maxTypeIdx = std::max(maxTypeIdx, a.getDeviceType());
    }
    if (startSpuEnabled && pathUpdaterPtr) {
        pathUpdaterPtr->initializeMatrix(std::max(1, maxTypeIdx + 1));
    }

    struct SegKey {
        int from;
        int to;
        int type;
        bool operator==(const SegKey& o) const noexcept {
            return from == o.from && to == o.to && type == o.type;
        }
    };
    struct SegKeyHash {
        size_t operator()(const SegKey& k) const noexcept {
            return std::hash<int>()(k.from) ^ (std::hash<int>()(k.to) << 1) ^ (std::hash<int>()(k.type) << 2);
        }
    };
    std::unordered_map<SegKey, double, SegKeyHash> segCache;

    auto query_cached = [&](int from, int to, int type) -> double {
        if (!startSpuEnabled || !pathUpdaterPtr) {
            return std::numeric_limits<double>::infinity();
        }
        if (!mapInfo.isConnectedById(from, to)) {
            return std::numeric_limits<double>::infinity();
        }
        SegKey key{from, to, type};
        auto it = segCache.find(key);
        if (it != segCache.end()) return it->second;
        double v = TravelSecondsGated(from, to, type, *pathUpdaterPtr, mapInfo, true, 300);
        segCache.emplace(key, v);
        return v;
    };
    std::vector<int> amrTypes;
    if (candidateAmrs.empty()) {
        amrTypes.push_back(0);
    } else {
        for (const auto& a : candidateAmrs) {
            amrTypes.push_back(a.getDeviceType());
        }
    }

    for (size_t idx = 0; idx < tasks.size(); ++idx) {
        const Task& task = tasks[idx];
        int failedFrom = -1;
        int failedTo = -1;
        std::string reason = "未知原因";
        if (!IsTaskReachable(task, mapInfo, amrTypes, query_cached, startSpuEnabled, failedFrom, failedTo, reason)) {
            result.forcedTaskIndices.push_back(static_cast<int>(idx));
            result.forcedReasons[idx] = reason;
            std::cout << "[TaskFilter][警告] 任务 " << task.getMessageId()
                      << " 不可达（" << reason << "），节点: " << failedFrom << " -> " << failedTo
                      << "，将标记为未分配" << std::endl;
            continue;
        }
        bool amrReachable = candidateAmrs.empty();
        TaskEndpoints ep = ResolveEndpoints(task);
        int targetNode = ep.start;
        if (!candidateAmrs.empty()) {
            for (const auto& amr : candidateAmrs) {
                int startNode = AmrPositionResolver::resolveStartNodeId(amr, mapInfo);
                if (startNode < 0) continue;
                if (targetNode >= 0 && mapInfo.isConnectedById(startNode, targetNode)) {
                    amrReachable = true;
                    break;
                }
            }
        }
        if (!amrReachable) {
            result.forcedTaskIndices.push_back(static_cast<int>(idx));
            result.forcedReasons[idx] = "所有 AMR 起点均不可达";
            std::cout << "[TaskFilter][警告] 任务 " << task.getMessageId()
                      << " 对所有 AMR 起点均不可达，将标记为未分配" << std::endl;
            continue;
        }
        result.originalToFiltered[idx] = static_cast<int>(result.filteredTasks.size());
        result.filteredTasks.push_back(task);
        result.filteredToOriginal.push_back(static_cast<int>(idx));
    }

    return result;
}

AllocationResult RemapAllocationToOriginal(
    const AllocationResult& filteredResult,
    const std::vector<int>& filteredToOriginal,
    size_t originalTaskCount) {
    AllocationResult mapped = filteredResult;

    auto parsed = filteredResult.amrTasks.empty()
        ? TaskAllocationUtils::parseSolution(filteredResult.code)
        : filteredResult.amrTasks;

    for (auto& chain : parsed) {
        for (int& idx : chain) {
            if (idx >= 0 && idx < static_cast<int>(filteredToOriginal.size())) {
                idx = filteredToOriginal[idx];
            } else {
                idx = -1;
            }
        }
    }
    mapped.amrTasks = parsed;
    mapped.code = TaskAllocationUtils::deParseSolution(mapped.amrTasks);

    mapped.unallocatedTaskIds.clear();
    for (int idx : filteredResult.unallocatedTaskIds) {
        if (idx >= 0 && idx < static_cast<int>(filteredToOriginal.size())) {
            int mappedIdx = filteredToOriginal[idx];
            if (mappedIdx >= 0 && mappedIdx < static_cast<int>(originalTaskCount)) {
                mapped.unallocatedTaskIds.push_back(mappedIdx);
            }
        }
    }

    return mapped;
}

void AppendForcedTasks(AllocationResult& result, const std::vector<int>& forcedTaskIndices) {
    for (int idx : forcedTaskIndices) {
        if (idx >= 0) {
            result.unallocatedTaskIds.push_back(idx);
        }
    }
    auto& ids = result.unallocatedTaskIds;
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
}

std::vector<int> InsertForcedTasks(
    AllocationResult& result,
    const std::vector<int>& forcedTaskIndices,
    const std::vector<int>& availableToFull,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& originalTasks,
    ShortestPathUpdater& pathUpdater,
    const MapInfo& mapInfo) {
    std::vector<int> pending;
    const bool finalSpuEnabled = ReadEnvBool("REACH_FINAL_SPU", false);
    if (forcedTaskIndices.empty() || availableToFull.empty()) {
        pending = forcedTaskIndices;
        return pending;
    }

    auto internal_cost_wait = [&](const Task& task, int amrType) -> double {
        auto nodes = CollectTaskNodes(task);
        if (nodes.size() <= 1) return 0.0;
        double total = 0.0;
        for (size_t i = 0; i + 1 < nodes.size(); ++i) {
            double seg = TravelSecondsGated(nodes[i], nodes[i + 1], amrType, pathUpdater, mapInfo, finalSpuEnabled, 500);
            if (!std::isfinite(seg)) return std::numeric_limits<double>::infinity();
            total += seg;
        }
        return total;
    };

    struct Candidate {
        int taskIdx;
        int priority;
        std::string startIso;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(forcedTaskIndices.size());
    for (int idx : forcedTaskIndices) {
        if (idx < 0 || idx >= static_cast<int>(originalTasks.size())) continue;
        candidates.push_back(Candidate{idx, originalTasks[idx].getPriority(), NormalizeStartIso(originalTasks[idx])});
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (a.priority != b.priority) return a.priority > b.priority;
        return a.startIso < b.startIso;
    });

    std::vector<int> availableStartNodes(amrList.size(), -1);
    for (size_t i = 0; i < amrList.size(); ++i) {
        availableStartNodes[i] = AmrPositionResolver::resolveStartNodeId(amrList[i], mapInfo);
    }

    for (const auto& cand : candidates) {
        const Task& task = originalTasks[cand.taskIdx];
        TaskEndpoints taskEp = ResolveEndpoints(task);
        if (taskEp.start < 0 || taskEp.end < 0) {
            pending.push_back(cand.taskIdx);
            continue;
        }
        double bestDelta = std::numeric_limits<double>::infinity();
        int bestFullIdx = -1;
        size_t bestPos = 0;
        for (size_t availIdx = 0; availIdx < availableToFull.size(); ++availIdx) {
            int fullIdx = availableToFull[availIdx];
            if (fullIdx < 0 || fullIdx >= static_cast<int>(result.amrTasks.size())) continue;
            const Amr& amr = amrList[fullIdx];
            int amrType = amr.getDeviceType();
            auto& chain = result.amrTasks[fullIdx];
            std::vector<TaskEndpoints> chainEndpoints(chain.size());
            bool chainValid = true;
            for (size_t j = 0; j < chain.size(); ++j) {
                if (chain[j] < 0 || chain[j] >= static_cast<int>(originalTasks.size())) {
                    chainValid = false;
                    break;
                }
                chainEndpoints[j] = ResolveEndpoints(originalTasks[chain[j]]);
            }
            if (!chainValid) continue;
            int prevExitNode = availableStartNodes[fullIdx];
            for (size_t pos = 0; pos <= chain.size(); ++pos) {
                int beforeNode = (pos == 0) ? prevExitNode : chainEndpoints[pos - 1].end;
                int afterNode = (pos < chain.size()) ? chainEndpoints[pos].start : -1;
                if (beforeNode < 0) continue;
                double removedCost = 0.0;
                if (pos < chain.size()) {
                    removedCost = TravelSecondsGated(beforeNode, afterNode, amrType, pathUpdater, mapInfo, finalSpuEnabled, 500);
                    if (!std::isfinite(removedCost)) continue;  // 原链路不可达，跳过该插入点
                }
                double costToNew = TravelSecondsGated(beforeNode, taskEp.start, amrType, pathUpdater, mapInfo, finalSpuEnabled, 500);
                if (!std::isfinite(costToNew)) continue;
                double costFromNew = 0.0;
                if (afterNode >= 0) {
                    costFromNew = TravelSecondsGated(taskEp.end, afterNode, amrType, pathUpdater, mapInfo, finalSpuEnabled, 500);
                    if (!std::isfinite(costFromNew)) continue;
                }
                double internalCost = internal_cost_wait(task, amrType);
                if (!std::isfinite(internalCost)) continue;
                double newCost = costToNew + internalCost + CalcServiceSec(task) + costFromNew;
                double delta = newCost - removedCost;
                if (delta < 0) {
                    // 在完备的道路信息下，插入任务不会减少成本。遇到负数说明原成本无效，跳过。
                    continue;
                }
                if (delta < bestDelta) {
                    bestDelta = delta;
                    bestFullIdx = fullIdx;
                    bestPos = pos;
                }
            }
        }
        if (bestFullIdx >= 0 && std::isfinite(bestDelta)) {
            auto& seq = result.amrTasks[bestFullIdx];
            seq.insert(seq.begin() + static_cast<long>(bestPos), cand.taskIdx);
            std::cout << "[TaskFilter] 任务 " << task.getMessageId()
                      << " 重新插入 AMR(" << amrList[bestFullIdx].getDeviceId()
                      << ") 位置 " << (bestPos + 1) << "，增量 " << bestDelta << " s" << std::endl;
        } else {
            pending.push_back(cand.taskIdx);
        }
    }

    result.code = TaskAllocationUtils::deParseSolution(result.amrTasks);
    return pending;
}

std::vector<double> ComputeChainDurationsSec(
    const AllocationResult& result,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& originalTasks,
    ShortestPathUpdater& pathUpdater,
    const MapInfo& mapInfo) {
    const bool finalSpuEnabled = ReadEnvBool("REACH_FINAL_SPU", false);
    std::vector<double> durations(result.amrTasks.size(), 0.0);
    size_t count = std::min(result.amrTasks.size(), amrList.size());
    for (size_t i = 0; i < count; ++i) {
        durations[i] = ChainDurationSec(result.amrTasks[i], amrList[i], originalTasks, pathUpdater, mapInfo, finalSpuEnabled);
    }
    return durations;
}

}  // namespace TaskReachabilityFilter
