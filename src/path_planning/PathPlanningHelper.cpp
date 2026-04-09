#include "path_planning/PathPlanningHelper.h"

#include <algorithm>
#include <unordered_map>
#include <set>
#include <iostream>
#include <cstdlib>
#include <optional>
#include <mutex>
#include "common/AmrPositionResolver.h"
#include "StaticPathTable.h"
#include "AStarPathFinder.h"
#include "data/MapInfo.h"

namespace PathPlanningHelper {

namespace {
struct TargetNodeInfo {
    int nodeId;
    std::string stepType;
};

std::vector<TargetNodeInfo> CollectTargetNodes(const Task& task) {
    std::vector<TargetNodeInfo> nodes;
    const auto& subs = task.getSubTasks();
    nodes.reserve(subs.size());
    if (!subs.empty()) {
        for (const auto& sub : subs) {
            int nodeId = sub.getPoint().getNodeId();
            if (nodeId < 0) continue;
            nodes.push_back(TargetNodeInfo{
                nodeId,
                std::to_string(sub.getPointType())
            });
        }
        if (!nodes.empty()) return nodes;
    }
    if (task.getStartId() >= 0) {
        nodes.push_back(TargetNodeInfo{task.getStartId(), std::string("AUTO_START")});
    }
    if (task.getEndId() >= 0 && task.getEndId() != task.getStartId()) {
        nodes.push_back(TargetNodeInfo{task.getEndId(), std::string("AUTO_END")});
    }
    return nodes;
}
}  // namespace

std::vector<std::vector<int>> ParseSolutionFallback(const std::vector<int>& code, size_t amrCount) {
    std::vector<std::vector<int>> parsed = TaskAllocationUtils::parseSolution(code);
    if (parsed.size() < amrCount) parsed.resize(amrCount);
    return parsed;
}

struct SegmentPlan {
    bool reachable = false;
    GlobalPathPlanner::PathSource source = GlobalPathPlanner::PathSource::UNREACHABLE;
    std::vector<int> nodes;
    double distanceMm = 0.0;
    double timeSec = 0.0;
};

SegmentPlan PlanSegment(int startNode,
                        int endNode,
                        StaticPathTable& staticTable,
                        AStarPathFinder& aStar,
                        const MapInfo& mapInfo) {
    SegmentPlan plan;
    if (startNode < 0 || endNode < 0) {
        return plan;
    }
    auto candidate = staticTable.query(startNode, endNode);
    if (candidate.has_value()) {
        plan.reachable = true;
        plan.source = GlobalPathPlanner::PathSource::STATIC_TABLE;
        plan.nodes = candidate->fullPath;
        plan.distanceMm = candidate->baseDistance;
        plan.timeSec = candidate->estimatedTime;
        return plan;
    }
    auto astarRes = aStar.findPath(startNode, endNode);
    if (astarRes.found && !astarRes.path.empty()) {
        plan.reachable = true;
        plan.source = GlobalPathPlanner::PathSource::PURE_ASTAR;
        plan.nodes = astarRes.path;
        plan.distanceMm = astarRes.distance;
        double speed = mapInfo.getGlobalMaxSpeed();
        if (!(speed > 0.0)) speed = 1000.0;
        plan.timeSec = plan.distanceMm / speed;
        return plan;
    }
    return plan;
}
std::vector<AmrPlanInfo> BuildAmrPathPlans(
    const AllocationResult& allocationResult,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    StaticPathTable& staticTable,
    AStarPathFinder& aStar,
    const MapInfo& mapInfo) {
    std::vector<std::vector<int>> assignments = allocationResult.amrTasks.empty()
        ? ParseSolutionFallback(allocationResult.code, amrList.size())
        : allocationResult.amrTasks;
    if (assignments.size() < amrList.size()) assignments.resize(amrList.size());

    std::vector<AmrPlanInfo> plans;
    size_t amrCount = std::min(assignments.size(), amrList.size());

    for (size_t amrIdx = 0; amrIdx < amrCount; ++amrIdx) {
        const auto& assignedTasks = assignments[amrIdx];
        if (assignedTasks.empty()) continue;

        AmrPlanInfo plan;
        plan.amrId = amrList[amrIdx].getDeviceId();
        // 统一AMR起点解析逻辑
        plan.startNodeId = AmrPositionResolver::resolveStartNodeId(amrList[amrIdx], mapInfo);
        int currentNode = plan.startNodeId;

        for (size_t taskOrder = 0; taskOrder < assignedTasks.size(); ++taskOrder) {
            int taskIdx = assignedTasks[taskOrder];
            if (taskIdx < 0 || taskIdx >= (int)taskList.size()) continue;
            const Task& task = taskList[taskIdx];
            const auto& subs = task.getSubTasks();
            auto targets = CollectTargetNodes(task);
            if (targets.empty()) continue;

            for (size_t subIdx = 0; subIdx < targets.size(); ++subIdx) {
                const auto& target = targets[subIdx];
                int targetNode = target.nodeId;
                PathSegmentInfo segment;
                segment.taskId = task.getMessageId();
                segment.stepType = target.stepType;
                int seqVal = -1;
                if (!subs.empty() && subIdx < subs.size()) {
                    seqVal = subs[subIdx].getSequence();
                }
                if (seqVal <= 0) seqVal = static_cast<int>(subIdx + 1);
                segment.subTaskSequence = seqVal;
                std::string segSubId;
                if (!subs.empty() && subIdx < subs.size()) {
                    segSubId = subs[subIdx].getSubTaskId();
                }
                if (segSubId.empty() && !segment.taskId.empty()) {
                    segSubId = segment.taskId + "#" + std::to_string(seqVal);
                }
                segment.subTaskId = std::move(segSubId);
                segment.fromNodeId = currentNode;
                segment.toNodeId = targetNode;
                double serviceTimeSec = 0.0;
                if (!subs.empty() && subIdx < subs.size()) {
                    serviceTimeSec = subs[subIdx].getEstimatedDurationSeconds();
                }
                segment.serviceTimeSec = serviceTimeSec;

                bool validNodes = (currentNode != -1 && targetNode != -1);
                if (validNodes) {
                    auto segPlan = PlanSegment(currentNode, targetNode, staticTable, aStar, mapInfo);
                    segment.nodes = segPlan.nodes;
                    segment.distanceMm = segPlan.distanceMm;
                    segment.timeSec = segPlan.timeSec;
                    segment.reachable = segPlan.reachable;
                    segment.source = segPlan.source;
                    plan.totalDistanceMm += segPlan.distanceMm;
                    plan.totalTimeSec += segPlan.timeSec;
                    plan.totalTimeSec += segment.serviceTimeSec;
                    if (!segPlan.reachable) plan.allReachable = false;
                } else {
                    segment.nodes.clear();
                    segment.distanceMm = 0.0;
                    segment.timeSec = 0.0;
                    segment.reachable = false;
                    segment.source = GlobalPathPlanner::PathSource::UNREACHABLE;
                    plan.allReachable = false;
                }

                // 描述信息：起点→取货 / 取货→放货
                std::string phase;
                if (!subs.empty()) {
                    if (subIdx == 0) {
                        if (taskOrder == 0) phase = "【起点→取货】";
                        else phase = "【上一终点→取货】";
                    } else {
                        phase = "【取货→放货】";
                    }
                } else {
                    phase = (subIdx == 0)
                        ? "【起点→任务节点】"
                        : "【任务节点→终点】";
                }
                segment.description = phase + " " + segment.taskId +
                    " (" + std::to_string(currentNode) + "->" + std::to_string(targetNode) + ")";

                plan.segments.push_back(std::move(segment));
                currentNode = targetNode;
            }
        }

        plans.push_back(std::move(plan));
    }
    return plans;
}

std::vector<AmrPlanInfo> BuildAmrPathPlans(
    const AllocationResult& allocationResult,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    GlobalPathPlanner& planner,
    const MapInfo& mapInfo) {
    std::vector<std::vector<int>> assignments = allocationResult.amrTasks.empty()
        ? ParseSolutionFallback(allocationResult.code, amrList.size())
        : allocationResult.amrTasks;
    if (assignments.size() < amrList.size()) assignments.resize(amrList.size());

    std::vector<AmrPlanInfo> plans;
    size_t amrCount = std::min(assignments.size(), amrList.size());

    for (size_t amrIdx = 0; amrIdx < amrCount; ++amrIdx) {
        const auto& assignedTasks = assignments[amrIdx];
        if (assignedTasks.empty()) continue;

        AmrPlanInfo plan;
        plan.amrId = amrList[amrIdx].getDeviceId();
        plan.startNodeId = AmrPositionResolver::resolveStartNodeId(amrList[amrIdx], mapInfo);
        int currentNode = plan.startNodeId;

        for (size_t taskOrder = 0; taskOrder < assignedTasks.size(); ++taskOrder) {
            int taskIdx = assignedTasks[taskOrder];
            if (taskIdx < 0 || taskIdx >= (int)taskList.size()) continue;
            const Task& task = taskList[taskIdx];
            const auto& subs = task.getSubTasks();
            auto targets = CollectTargetNodes(task);
            if (targets.empty()) continue;

            for (size_t subIdx = 0; subIdx < targets.size(); ++subIdx) {
                const auto& target = targets[subIdx];
                int targetNode = target.nodeId;
                PathSegmentInfo segment;
                segment.taskId = task.getMessageId();
                segment.stepType = target.stepType;
                int seqVal = -1;
                if (!subs.empty() && subIdx < subs.size()) {
                    seqVal = subs[subIdx].getSequence();
                }
                if (seqVal <= 0) seqVal = static_cast<int>(subIdx + 1);
                segment.subTaskSequence = seqVal;
                std::string segSubId;
                if (!subs.empty() && subIdx < subs.size()) {
                    segSubId = subs[subIdx].getSubTaskId();
                }
                if (segSubId.empty() && !segment.taskId.empty()) {
                    segSubId = segment.taskId + "#" + std::to_string(seqVal);
                }
                segment.subTaskId = std::move(segSubId);
                segment.fromNodeId = currentNode;
                segment.toNodeId = targetNode;
                double serviceTimeSec = 0.0;
                if (!subs.empty() && subIdx < subs.size()) {
                    serviceTimeSec = subs[subIdx].getEstimatedDurationSeconds();
                }
                segment.serviceTimeSec = serviceTimeSec;

                bool validNodes = (currentNode != -1 && targetNode != -1);
                if (validNodes) {
                    auto res = planner.planGlobalPath(currentNode, targetNode);
                    segment.nodes = !res.fullPath.empty() ? res.fullPath : res.path;
                    segment.distanceMm = res.distance;
                    segment.timeSec = res.estimatedTime;
                    segment.reachable = res.reachable;
                    segment.source = res.source;
                    plan.totalDistanceMm += res.distance;
                    plan.totalTimeSec += res.estimatedTime;
                    plan.totalTimeSec += segment.serviceTimeSec;
                    if (!res.reachable) plan.allReachable = false;
                } else {
                    segment.nodes.clear();
                    segment.distanceMm = 0.0;
                    segment.timeSec = 0.0;
                    segment.reachable = false;
                    segment.source = GlobalPathPlanner::PathSource::UNREACHABLE;
                    plan.allReachable = false;
                }

                std::string phase;
                if (!subs.empty()) {
                    if (subIdx == 0) {
                        phase = (taskOrder == 0) ? "【起点→取货】" : "【上一终点→取货】";
                    } else {
                        phase = "【取货→放货】";
                    }
                } else {
                    phase = (subIdx == 0)
                        ? "【起点→任务节点】"
                        : "【任务节点→终点】";
                }
                segment.description = phase + " " + segment.taskId +
                    " (" + std::to_string(currentNode) + "->" + std::to_string(targetNode) + ")";

                plan.segments.push_back(std::move(segment));
                currentNode = targetNode;
            }
        }

        plans.push_back(std::move(plan));
    }
    return plans;
}

}  // namespace PathPlanningHelper
