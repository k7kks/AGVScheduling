#include "common/CongestionRegionUtils.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "common/PathPlanningConstants.h"

namespace CongestionRegionUtils {

int autoRegionGrid(int itemCount, int minGrid, int maxGrid) {
    if (minGrid < 1) minGrid = 1;
    if (maxGrid < minGrid) maxGrid = minGrid;
    int grid = static_cast<int>(std::lround(std::sqrt(std::max(1, itemCount))));
    return std::clamp(grid, minGrid, maxGrid);
}

bool isPassableRegionNode(const Node& node) {
    if (node.id < 0) return false;
    if (node.type == -1) return false;
    if (node.allowPass == 0) return false;
    return true;
}

void gatherNeighborIndices(const MapInfo& mapInfo, int idx, std::vector<int>& out) {
    out.clear();
    const auto& aft = mapInfo.getAftNode();
    const auto& pre = mapInfo.getPreNode();
    auto itA = aft.find(idx);
    if (itA != aft.end()) {
        out.insert(out.end(), itA->second.begin(), itA->second.end());
    }
    auto itP = pre.find(idx);
    if (itP != pre.end()) {
        out.insert(out.end(), itP->second.begin(), itP->second.end());
    }
    if (out.size() > 1) {
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    }
}

namespace {

int countPassableRegionNeighbors(const MapInfo& mapInfo,
                                 const std::vector<Node>& nodes,
                                 int idx) {
    std::vector<int> uniq;
    gatherNeighborIndices(mapInfo, idx, uniq);
    int count = 0;
    for (int nb : uniq) {
        if (nb < 0 || nb >= static_cast<int>(nodes.size())) continue;
        if (!isPassableRegionNode(nodes[static_cast<size_t>(nb)])) continue;
        count += 1;
    }
    return count;
}

void collectBridgeRegionGroups(const MapInfo& mapInfo,
                               std::vector<std::vector<int>>& groups) {
    groups.clear();
    const auto& nodes = mapInfo.getNodes();
    const int nodeCount = static_cast<int>(nodes.size());
    if (nodeCount <= 0) return;

    std::vector<char> bridgeCore(static_cast<size_t>(nodeCount), 0);
    for (int idx = 0; idx < nodeCount; ++idx) {
        const Node& node = nodes[static_cast<size_t>(idx)];
        if (!isPassableRegionNode(node)) continue;
        int undirectedDegree = countPassableRegionNeighbors(mapInfo, nodes, idx);
        if (undirectedDegree == 2) {
            bridgeCore[static_cast<size_t>(idx)] = 1;
        }
    }

    std::vector<int> componentByIndex(static_cast<size_t>(nodeCount), -1);
    std::vector<std::vector<int>> coreNodesByComponent;
    std::vector<int> neighbors;
    for (int idx = 0; idx < nodeCount; ++idx) {
        if (!bridgeCore[static_cast<size_t>(idx)]) continue;
        if (componentByIndex[static_cast<size_t>(idx)] >= 0) continue;
        int compId = static_cast<int>(coreNodesByComponent.size());
        coreNodesByComponent.push_back({});
        std::deque<int> q;
        q.push_back(idx);
        componentByIndex[static_cast<size_t>(idx)] = compId;
        while (!q.empty()) {
            int cur = q.front();
            q.pop_front();
            coreNodesByComponent[static_cast<size_t>(compId)].push_back(cur);
            gatherNeighborIndices(mapInfo, cur, neighbors);
            for (int nb : neighbors) {
                if (nb < 0 || nb >= nodeCount) continue;
                if (!bridgeCore[static_cast<size_t>(nb)]) continue;
                if (componentByIndex[static_cast<size_t>(nb)] >= 0) continue;
                componentByIndex[static_cast<size_t>(nb)] = compId;
                q.push_back(nb);
            }
        }
    }
    if (coreNodesByComponent.empty()) return;

    const int minCoreNodes = std::max(1, PathPlanningConstants::resolveBridgeRegionMinNodes());
    std::vector<std::pair<int, std::vector<int>>> orderedGroups;
    orderedGroups.reserve(coreNodesByComponent.size());
    for (size_t compIdx = 0; compIdx < coreNodesByComponent.size(); ++compIdx) {
        const auto& coreNodes = coreNodesByComponent[compIdx];
        if (static_cast<int>(coreNodes.size()) < minCoreNodes) continue;

        std::unordered_set<int> coreSet(coreNodes.begin(), coreNodes.end());
        std::vector<int> endpointCoreNodes;
        endpointCoreNodes.reserve(coreNodes.size());
        for (int idx : coreNodes) {
            gatherNeighborIndices(mapInfo, idx, neighbors);
            int coreNeighborCount = 0;
            for (int nb : neighbors) {
                if (coreSet.count(nb) > 0) {
                    coreNeighborCount += 1;
                }
            }
            if (coreNeighborCount <= 1) {
                endpointCoreNodes.push_back(idx);
            }
        }

        std::vector<int> merged = coreNodes;
        for (int idx : endpointCoreNodes) {
            gatherNeighborIndices(mapInfo, idx, neighbors);
            for (int nb : neighbors) {
                if (nb < 0 || nb >= nodeCount) continue;
                if (!isPassableRegionNode(nodes[static_cast<size_t>(nb)])) continue;
                if (coreSet.count(nb) > 0) continue;
                merged.push_back(nb);
            }
        }
        std::sort(merged.begin(), merged.end());
        merged.erase(std::unique(merged.begin(), merged.end()), merged.end());
        orderedGroups.push_back({merged.front(), std::move(merged)});
    }

    std::sort(orderedGroups.begin(), orderedGroups.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
    groups.reserve(orderedGroups.size());
    for (auto& item : orderedGroups) {
        groups.push_back(std::move(item.second));
    }
}

}  // namespace

int buildGraphRegions(const MapInfo& mapInfo,
                      int targetRegions,
                      std::vector<int>& regionByIndex,
                      std::vector<int>& seedIndices,
                      std::vector<char>* bridgeRegionMask,
                      bool enableBridgeRegions) {
    const auto& nodes = mapInfo.getNodes();
    const int nodeCount = static_cast<int>(nodes.size());
    regionByIndex.assign(static_cast<size_t>(nodeCount), -1);
    seedIndices.clear();
    if (bridgeRegionMask) bridgeRegionMask->clear();
    if (targetRegions <= 0 || nodeCount <= 0) return 0;

    int regionBase = 0;
    if (enableBridgeRegions) {
        std::vector<std::vector<int>> bridgeGroups;
        collectBridgeRegionGroups(mapInfo, bridgeGroups);
        for (const auto& group : bridgeGroups) {
            if (group.empty()) continue;
            for (int idx : group) {
                if (idx < 0 || idx >= nodeCount) continue;
                regionByIndex[static_cast<size_t>(idx)] = regionBase;
            }
            seedIndices.push_back(group.front());
            if (bridgeRegionMask) bridgeRegionMask->push_back(1);
            regionBase += 1;
        }
    }

    std::vector<int> candidates;
    candidates.reserve(nodes.size());
    for (int i = 0; i < nodeCount; ++i) {
        if (!isPassableRegionNode(nodes[static_cast<size_t>(i)])) continue;
        if (regionByIndex[static_cast<size_t>(i)] >= 0) continue;
        candidates.push_back(i);
    }
    if (candidates.empty()) return regionBase;

    int remainingTargetRegions = targetRegions;
    if (regionBase > 0) {
        remainingTargetRegions = targetRegions - regionBase;
        if (remainingTargetRegions <= 0) remainingTargetRegions = 1;
    }
    if (remainingTargetRegions > static_cast<int>(candidates.size())) {
        remainingTargetRegions = static_cast<int>(candidates.size());
    }

    int first = candidates.front();
    double firstScore = nodes[static_cast<size_t>(first)].x + nodes[static_cast<size_t>(first)].y;
    for (int idx : candidates) {
        const Node& node = nodes[static_cast<size_t>(idx)];
        double score = node.x + node.y;
        if (score < firstScore) {
            firstScore = score;
            first = idx;
        }
    }
    std::vector<int> normalSeedIndices;
    normalSeedIndices.reserve(static_cast<size_t>(remainingTargetRegions));
    normalSeedIndices.push_back(first);

    while (static_cast<int>(normalSeedIndices.size()) < remainingTargetRegions) {
        int bestIdx = -1;
        double bestDist = -1.0;
        for (int idx : candidates) {
            const Node& node = nodes[static_cast<size_t>(idx)];
            double minDist = std::numeric_limits<double>::infinity();
            for (int seedIdx : normalSeedIndices) {
                const Node& seed = nodes[static_cast<size_t>(seedIdx)];
                double dx = node.x - seed.x;
                double dy = node.y - seed.y;
                double d2 = dx * dx + dy * dy;
                if (d2 < minDist) minDist = d2;
            }
            if (minDist > bestDist) {
                bestDist = minDist;
                bestIdx = idx;
            }
        }
        if (bestIdx < 0) break;
        normalSeedIndices.push_back(bestIdx);
    }

    seedIndices.insert(seedIndices.end(), normalSeedIndices.begin(), normalSeedIndices.end());
    if (bridgeRegionMask) {
        bridgeRegionMask->insert(bridgeRegionMask->end(),
                                 static_cast<size_t>(normalSeedIndices.size()),
                                 static_cast<char>(0));
    }

    std::vector<char> candidateMask(static_cast<size_t>(nodeCount), 0);
    for (int idx : candidates) {
        candidateMask[static_cast<size_t>(idx)] = 1;
    }

    std::deque<int> q;
    for (size_t r = 0; r < normalSeedIndices.size(); ++r) {
        int idx = normalSeedIndices[r];
        regionByIndex[static_cast<size_t>(idx)] = regionBase + static_cast<int>(r);
        q.push_back(idx);
    }

    std::vector<int> neighbors;
    while (!q.empty()) {
        int cur = q.front();
        q.pop_front();
        const int curRegion = regionByIndex[static_cast<size_t>(cur)];
        gatherNeighborIndices(mapInfo, cur, neighbors);
        for (int nb : neighbors) {
            if (nb < 0 || nb >= nodeCount) continue;
            if (!candidateMask[static_cast<size_t>(nb)]) continue;
            if (!isPassableRegionNode(nodes[static_cast<size_t>(nb)])) continue;
            if (regionByIndex[static_cast<size_t>(nb)] >= 0) continue;
            regionByIndex[static_cast<size_t>(nb)] = curRegion;
            q.push_back(nb);
        }
    }

    for (int idx : candidates) {
        if (regionByIndex[static_cast<size_t>(idx)] >= 0) continue;
        int bestRegion = regionBase;
        double bestDist = std::numeric_limits<double>::infinity();
        const Node& node = nodes[static_cast<size_t>(idx)];
        for (size_t r = 0; r < normalSeedIndices.size(); ++r) {
            const Node& seed = nodes[static_cast<size_t>(normalSeedIndices[r])];
            double dx = node.x - seed.x;
            double dy = node.y - seed.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < bestDist) {
                bestDist = d2;
                bestRegion = regionBase + static_cast<int>(r);
            }
        }
        regionByIndex[static_cast<size_t>(idx)] = bestRegion;
    }

    return regionBase + static_cast<int>(normalSeedIndices.size());
}

}  // namespace CongestionRegionUtils
