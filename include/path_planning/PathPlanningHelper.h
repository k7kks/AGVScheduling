#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

#include <string>
#include <vector>

#include "algorithm/base/TaskAllocatorBase.h"
#include "data/Amr.h"
#include "data/Task.h"
#include "GlobalPathPlanner.h"  // 只保留 PathSource 枚举复用

class StaticPathTable;
class AStarPathFinder;
class MapInfo;

namespace PathPlanningHelper {

struct PathSegmentInfo {
    std::string taskId;
    std::string stepType;
    std::string subTaskId;
    int subTaskSequence = -1;
    int fromNodeId = -1;
    int toNodeId = -1;
    std::vector<int> nodes;
    double distanceMm = 0.0;
    // 规划时间（秒）。注意：与 GlobalPathPlanner::PathResult::estimatedTime 单位一致。
    double timeSec = 0.0;
    // 子任务停留时间（秒）
    double serviceTimeSec = 0.0;
    GlobalPathPlanner::PathSource source = GlobalPathPlanner::PathSource::UNREACHABLE;
    bool reachable = false;
    std::string description;
};

struct AmrPlanInfo {
    std::string amrId;
    int startNodeId = -1;
    std::vector<PathSegmentInfo> segments;
    double totalDistanceMm = 0.0;
    double totalTimeSec = 0.0;
    bool allReachable = true;
};

std::vector<AmrPlanInfo> BuildAmrPathPlans(
    const AllocationResult& allocationResult,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    StaticPathTable& staticTable,
    AStarPathFinder& aStar,
    const MapInfo& mapInfo);

std::vector<AmrPlanInfo> BuildAmrPathPlans(
    const AllocationResult& allocationResult,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    GlobalPathPlanner& planner,
    const MapInfo& mapInfo);

}  // namespace PathPlanningHelper

#endif  // PATH_PLANNING_HELPER_H
