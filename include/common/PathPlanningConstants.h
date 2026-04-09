#pragma once

#include <cmath>
#include <cstdlib>
#include <vector>

#include "data/MapInfo.h"

namespace PathPlanningConstants {

// 默认转弯惩罚（单位：毫米），与路径规划模块保持一致
inline constexpr double kDefaultTurnPenaltyMm = 3000.0;
inline constexpr double kUTurnPenaltyMultiplier = 2.0;
inline constexpr double kDefaultPlannerSpeedMmPerSec = 1200.0;
inline constexpr int kMinBridgeRegionNodes = 4;
inline constexpr const char* kTurnPenaltyEnvKey = "PLANNER_TURN_PENALTY_MM";

inline double resolveTurnPenaltyMm() {
    if (const char* raw = std::getenv(kTurnPenaltyEnvKey)) {
        char* end = nullptr;
        const double value = std::strtod(raw, &end);
        if (end != raw && std::isfinite(value) && value >= 0.0) {
            return value;
        }
    }
    return kDefaultTurnPenaltyMm;
}

inline double resolvePlannerSpeedMmPerSec(double preferredMmPerSec) {
    if (std::isfinite(preferredMmPerSec) && preferredMmPerSec > 0.0) {
        return preferredMmPerSec;
    }
    return kDefaultPlannerSpeedMmPerSec;
}

inline double calculateTurnPenaltyMm(const Node& prev,
                                     const Node& curr,
                                     const Node& next,
                                     double basePenaltyMm) {
    const double dx1 = curr.x - prev.x;
    const double dy1 = curr.y - prev.y;
    const double dx2 = next.x - curr.x;
    const double dy2 = next.y - curr.y;

    double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    if (len1 < 1e-6 || len2 < 1e-6) {
        return 0.0;
    }

    const double ndx1 = dx1 / len1;
    const double ndy1 = dy1 / len1;
    const double ndx2 = dx2 / len2;
    const double ndy2 = dy2 / len2;
    const double cross = ndx1 * ndy2 - ndy1 * ndx2;
    const double dot = ndx1 * ndx2 + ndy1 * ndy2;

    if (dot < -0.999) {
        return basePenaltyMm * kUTurnPenaltyMultiplier;
    }
    if (std::abs(cross) > 1e-3) {
        return basePenaltyMm;
    }
    return 0.0;
}

inline int countTurnsByIndices(const std::vector<int>& pathIndices, const MapInfo& mapInfo) {
    if (pathIndices.size() < 3) {
        return 0;
    }

    const auto& nodes = mapInfo.getNodes();
    int turnCount = 0;
    for (size_t i = 1; i + 1 < pathIndices.size(); ++i) {
        const int prevIdx = pathIndices[i - 1];
        const int currIdx = pathIndices[i];
        const int nextIdx = pathIndices[i + 1];
        if (prevIdx < 0 || currIdx < 0 || nextIdx < 0) {
            continue;
        }
        if (prevIdx >= static_cast<int>(nodes.size()) ||
            currIdx >= static_cast<int>(nodes.size()) ||
            nextIdx >= static_cast<int>(nodes.size())) {
            continue;
        }
        if (calculateTurnPenaltyMm(nodes[prevIdx], nodes[currIdx], nodes[nextIdx], 1.0) > 0.0) {
            turnCount += 1;
        }
    }
    return turnCount;
}

inline double calculatePathTurnPenaltyByIndices(const std::vector<int>& pathIndices,
                                                const MapInfo& mapInfo,
                                                double basePenaltyMm) {
    if (pathIndices.size() < 3) {
        return 0.0;
    }

    const auto& nodes = mapInfo.getNodes();
    double totalPenaltyMm = 0.0;
    for (size_t i = 1; i + 1 < pathIndices.size(); ++i) {
        const int prevIdx = pathIndices[i - 1];
        const int currIdx = pathIndices[i];
        const int nextIdx = pathIndices[i + 1];
        if (prevIdx < 0 || currIdx < 0 || nextIdx < 0) {
            continue;
        }
        if (prevIdx >= static_cast<int>(nodes.size()) ||
            currIdx >= static_cast<int>(nodes.size()) ||
            nextIdx >= static_cast<int>(nodes.size())) {
            continue;
        }
        totalPenaltyMm += calculateTurnPenaltyMm(nodes[prevIdx], nodes[currIdx], nodes[nextIdx], basePenaltyMm);
    }
    return totalPenaltyMm;
}

inline int countTurnsByNodeIds(const std::vector<int>& pathNodeIds, const MapInfo& mapInfo) {
    if (pathNodeIds.size() < 3) {
        return 0;
    }

    const auto& id2index = mapInfo.getId2Index();
    std::vector<int> indices;
    indices.reserve(pathNodeIds.size());
    for (int nodeId : pathNodeIds) {
        auto it = id2index.find(nodeId);
        if (it == id2index.end()) {
            indices.push_back(-1);
        } else {
            indices.push_back(it->second);
        }
    }
    return countTurnsByIndices(indices, mapInfo);
}

inline double calculatePathTurnPenaltyByNodeIds(const std::vector<int>& pathNodeIds,
                                                const MapInfo& mapInfo,
                                                double basePenaltyMm) {
    if (pathNodeIds.size() < 3) {
        return 0.0;
    }

    const auto& id2index = mapInfo.getId2Index();
    std::vector<int> indices;
    indices.reserve(pathNodeIds.size());
    for (int nodeId : pathNodeIds) {
        auto it = id2index.find(nodeId);
        if (it == id2index.end()) {
            indices.push_back(-1);
        } else {
            indices.push_back(it->second);
        }
    }
    return calculatePathTurnPenaltyByIndices(indices, mapInfo, basePenaltyMm);
}

}  // namespace PathPlanningConstants
