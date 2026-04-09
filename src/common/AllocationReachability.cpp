#include "common/AllocationReachability.h"

#include <algorithm>
#include <set>
#include <limits>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <string>

#include "algorithm/base/TaskAllocationUtils.h"
#include "common/AmrPositionResolver.h"

namespace {

static std::pair<int,int> ExtractTaskEndpoints(const Task& task) {
    int startId = -1;
    int endId = -1;
    const auto& subs = task.getSubTasks();
    if (!subs.empty()) {
        startId = subs.front().getPoint().getNodeId();
        endId = subs.back().getPoint().getNodeId();
    } else {
        startId = task.getStartId();
        endId = task.getEndId();
    }
    return {startId, endId};
}

static int DetermineAmrStartNodeId(const Amr& amr, const MapInfo& mapInfo) {
    // 统一到 AmrPositionResolver 逻辑
    return AmrPositionResolver::resolveStartNodeId(amr, mapInfo);
}

static bool ReadEnvBool(const char* key, bool defv) {
    if (const char* v = std::getenv(key)) {
        std::string s(v);
        if (s == "0" || s == "false" || s == "False" || s == "FALSE") return false;
        return true;
    }
    return defv;
}

static double straight_line_sec(int fromNode, int toNode, const MapInfo& mapInfo) {
    if (fromNode < 0 || toNode < 0) return std::numeric_limits<double>::infinity();
    try {
        const Node& nf = mapInfo.getNodeById(fromNode);
        const Node& nt = mapInfo.getNodeById(toNode);
        double dx = nf.x - nt.x;
        double dy = nf.y - nt.y;
        double dist = std::abs(dx) + std::abs(dy);
        double speed = mapInfo.getGlobalMaxSpeed();
        if (speed <= 0) speed = 1000.0;
        return dist / speed; // 秒
    } catch (...) {
        return std::numeric_limits<double>::infinity();
    }
}

// 秒值路径查询；回退曼哈顿/全局速度近似
static double sp_seconds_or_approx(int fromNode, int toNode, int amrType,
                                   const MapInfo& mapInfo,
                                   const ShortestPathUpdater& pathUpdater,
                                   bool useSpu) {
    if (fromNode < 0 || toNode < 0) return std::numeric_limits<double>::infinity();
    if (!mapInfo.isConnectedById(fromNode, toNode)) return std::numeric_limits<double>::infinity();
    if (!useSpu) {
        return straight_line_sec(fromNode, toNode, mapInfo);
    }
    double sec = pathUpdater.queryByNodeId(fromNode, toNode, amrType);
    if (sec == static_cast<double>(DistanceState::UNREACHABLE)) return std::numeric_limits<double>::infinity();
    if (sec == static_cast<double>(DistanceState::UNCALCULATED) || !(sec > 0.0)) {
        return straight_line_sec(fromNode, toNode, mapInfo);
    }
    return sec; // 秒
}

} // namespace

AllocationResult SanitizeAllocationResult(
    const AllocationResult& result,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    const MapInfo& mapInfo,
    const ShortestPathUpdater& pathUpdater
) {
    const bool useSpuFinal = ReadEnvBool("REACH_FINAL_SPU", false);
    if (const char* dbg = std::getenv("DEBUG_SANITIZE")) {
        if (dbg && std::string(dbg) != "0") {
            std::cout << "[Sanitize] enter: amr=" << amrList.size()
                      << " tasks=" << taskList.size() << std::endl;
        }
    }
    AllocationResult out = result;
    // 解析/对齐 amrTasks
    auto parsed = result.amrTasks.empty()
        ? TaskAllocationUtils::parseSolution(result.code)
        : result.amrTasks;
    if (parsed.size() < amrList.size()) parsed.resize(amrList.size());

    std::vector<std::vector<int>> checked(parsed.size());
    std::vector<int> removed;

    // 逐 AMR 可达性过滤
    for (size_t ai=0; ai<parsed.size() && ai<amrList.size(); ++ai) {
        int curNode = DetermineAmrStartNodeId(amrList[ai], mapInfo);
        int amrType = amrList[ai].getDeviceType();
        for (int tidx : parsed[ai]) {
            if (tidx < 0 || tidx >= static_cast<int>(taskList.size())) continue;
            auto [sId, eId] = ExtractTaskEndpoints(taskList[tidx]);
            bool ok = std::isfinite(sp_seconds_or_approx(curNode, sId, amrType, mapInfo, pathUpdater, useSpuFinal));
            if (ok && sId >= 0 && eId >= 0 && sId != eId) {
                ok = std::isfinite(sp_seconds_or_approx(sId, eId, amrType, mapInfo, pathUpdater, useSpuFinal));
            }
            if (ok) {
                checked[ai].push_back(tidx);
                curNode = eId;
            } else {
                removed.push_back(tidx);
            }
        }
    }
    // 保留多余行（若 parsed 比 amrList 更长）
    for (size_t ai=amrList.size(); ai<parsed.size(); ++ai) {
        checked[ai] = parsed[ai];
    }

    // 尝试重新插入被移除任务（最小增量）
    std::vector<char> inserted(taskList.size(), 0);
    for (int tidx : removed) {
        if (tidx < 0 || tidx >= static_cast<int>(taskList.size())) continue;
        auto [sId, eId] = ExtractTaskEndpoints(taskList[tidx]);
        double bestDelta = std::numeric_limits<double>::infinity();
        size_t bestAmr = checked.size();
        size_t bestPos = 0;
        for (size_t ai=0; ai<checked.size() && ai<amrList.size(); ++ai) {
            auto& chain = checked[ai];
            int amrType = amrList[ai].getDeviceType();
            int startNode = DetermineAmrStartNodeId(amrList[ai], mapInfo);
            for (size_t pos=0; pos<=chain.size(); ++pos) {
                int prevEnd = (pos==0)? startNode : ExtractTaskEndpoints(taskList[chain[pos-1]]).second;
                int nextStart = (pos<chain.size())? ExtractTaskEndpoints(taskList[chain[pos]]).first : -1;
                double c1 = sp_seconds_or_approx(prevEnd, sId, amrType, mapInfo, pathUpdater, useSpuFinal);
                double c2 = sp_seconds_or_approx(sId, eId, amrType, mapInfo, pathUpdater, useSpuFinal);
                double c3 = (nextStart>=0)? sp_seconds_or_approx(eId, nextStart, amrType, mapInfo, pathUpdater, useSpuFinal) : 0.0;
                if (!std::isfinite(c1) || !std::isfinite(c2) || !std::isfinite(c3)) continue;
                double base = (nextStart>=0)? sp_seconds_or_approx(prevEnd, nextStart, amrType, mapInfo, pathUpdater, useSpuFinal) : 0.0;
                if (!std::isfinite(base)) base = 0.0;
                double delta = c1 + c2 + c3 - base;
                if (delta < bestDelta) { bestDelta = delta; bestAmr = ai; bestPos = pos; }
            }
        }
        if (bestAmr < checked.size() && std::isfinite(bestDelta)) {
            auto& chain = checked[bestAmr];
            chain.insert(chain.begin() + static_cast<long>(bestPos), tidx);
            inserted[tidx] = 1;
        }
    }

    // 合并未分配列表并去重
    std::vector<char> unallocFlag(taskList.size(), 0);
    for (int id : out.unallocatedTaskIds) if (id>=0 && id < static_cast<int>(taskList.size())) unallocFlag[id] = 1;
    int reinserts = 0;
    for (int tidx : removed) {
        if (tidx >= 0 && tidx < static_cast<int>(taskList.size()) && inserted[tidx]) ++reinserts;
    }
    for (int tidx : removed) {
        if (tidx < 0 || tidx >= static_cast<int>(taskList.size())) continue;
        if (!inserted[tidx] && !unallocFlag[tidx]) {
            out.unallocatedTaskIds.push_back(tidx);
            unallocFlag[tidx] = 1;
        }
    }
    if (!out.unallocatedTaskIds.empty()) {
        std::sort(out.unallocatedTaskIds.begin(), out.unallocatedTaskIds.end());
        out.unallocatedTaskIds.erase(std::unique(out.unallocatedTaskIds.begin(), out.unallocatedTaskIds.end()), out.unallocatedTaskIds.end());
    }

    // 更新结果与日志
    out.amrTasks = checked;
    out.code = TaskAllocationUtils::deParseSolution(out.amrTasks);
    int dropped = static_cast<int>(removed.size()) - reinserts;
    if (!removed.empty()) {
        out.repairLog = "reachability_removed=" + std::to_string(removed.size()) +
                        " reinserted=" + std::to_string(reinserts) +
                        " dropped=" + std::to_string(dropped);
    } else {
        // 不覆盖已有日志
        if (out.repairLog.empty()) out.repairLog.clear();
    }

    // 重新计算 totalCost（使用可用AMR数）
    out.totalCost = TaskAllocationUtils::calculateTotalCost(
        static_cast<int>(amrList.size()), static_cast<int>(taskList.size()), out.code);

    return out;
}
