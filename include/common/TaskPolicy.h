#ifndef TASK_POLICY_H
#define TASK_POLICY_H

#include <vector>
#include <cmath>
#include "data/Amr.h"
#include "data/Task.h"
#include "data/MapInfo.h"

namespace TaskPolicy {

// 紧急级别与任务数量阈值（可按需调整）
constexpr int EMERGENCY_LEVEL = 10;
constexpr size_t MAX_TASK_NUM_TH = 10;

inline bool hasEmergency(const std::vector<Task>& tasks) {
    for (const auto& t : tasks) if (t.getPriority() >= EMERGENCY_LEVEL) return true;
    return false;
}

inline bool hasIdleAmr(const std::vector<Amr>& amrs) {
    for (const auto& a : amrs) if (a.isIdle()) return true;
    return false;
}

inline double minkowskiL1(double x1, double y1, double x2, double y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

// 根据状态与阈值筛选可用AMR
inline std::vector<Amr> selectAvailableAmrs(const std::vector<Amr>& amrs,
                                            const std::vector<Task>& tasks,
                                            const MapInfo& mapInfo) {
    std::vector<Amr> out;
    // 若无任务，直接返回空（避免不必要分配）
    if (tasks.empty()) return out;

    // 取第一个任务的起点坐标
    const Task& t0 = tasks.front();
    int startId = t0.getStartId();
    int sx = 0, sy = 0;
    try {
        const Node& sn = mapInfo.getNodeById(startId);
        sx = (int)std::round(sn.x); sy = (int)std::round(sn.y);
    } catch(...) { sx = 0; sy = 0; }

    // 是否存在紧急（高优先级）任务：仅在此时允许“工作中/移动中”的AMR参与再分配
    bool emergency = hasEmergency(tasks);
    for (const auto& a : amrs) {
        if (a.isUnavailable()) continue; // 失败/未执行/暂停/不接受任务不参与分配
        // 空闲AMR始终可以参与分配
        if (a.isIdle()) { out.push_back(a); continue; }
        // 非紧急场景下，不打扰“工作中/移动中”的AMR
        if (!emergency) continue;
        // 存在紧急任务时，才允许工作中/移动中的AMR参与分配
        if (a.isMoving()) {
            double d = minkowskiL1(a.getX(), a.getY(), sx, sy);
            if ((int)d >= a.getDisThresholdMm()) out.push_back(a);
            continue;
        }
        if (a.isWorking()) {
            // 允许参与（成本矩阵位置选取为下一目标点/邻域）
            out.push_back(a);
            continue;
        }
    }
    return out;
}

inline bool shouldPlan(const std::vector<Amr>& amrs, const std::vector<Task>& tasks) {
    bool hasTask = !tasks.empty();
    bool idle = hasIdleAmr(amrs);
    bool emergency = hasEmergency(tasks);
    if (!hasTask) return false;
    if (idle || emergency) return true;
    return false;
}

} // namespace TaskPolicy

#endif // TASK_POLICY_H
