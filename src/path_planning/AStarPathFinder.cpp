#include "AStarPathFinder.h"
#include <cmath>
#include <algorithm>

AStarPathFinder::AStarPathFinder(const MapInfo& mapInfo)
    : mapInfo_(mapInfo)
{
    buildNodeMapping();
}

void AStarPathFinder::buildNodeMapping() {
    const auto& nodes = mapInfo_.getNodes();
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        int nodeId = nodes[i].id;
        nodeIdToIndex_[nodeId] = i;
        nodeIndexToId_[i] = nodeId;
    }
}

double AStarPathFinder::heuristic(int fromIdx, int toIdx) const {
    const auto& nodes = mapInfo_.getNodes();
    
    if (fromIdx < 0 || fromIdx >= static_cast<int>(nodes.size()) ||
        toIdx < 0 || toIdx >= static_cast<int>(nodes.size())) {
        return std::numeric_limits<double>::infinity();
    }
    
    const Node& from = nodes[fromIdx];
    const Node& to = nodes[toIdx];
    
    double dx = std::abs(to.x - from.x);
    double dy = std::abs(to.y - from.y);
    double straightDistance = dx + dy;
    
    return straightDistance;
}

double AStarPathFinder::calculateTurnPenalty(
    int prevIdx, int currIdx, int nextIdx, double penalty
) const {
    const auto& nodes = mapInfo_.getNodes();
    
    if (prevIdx < 0 || currIdx < 0 || nextIdx < 0 ||
        prevIdx >= static_cast<int>(nodes.size()) ||
        currIdx >= static_cast<int>(nodes.size()) ||
        nextIdx >= static_cast<int>(nodes.size())) {
        return 0.0;
    }
    
    return PathPlanningConstants::calculateTurnPenaltyMm(
        nodes[prevIdx],
        nodes[currIdx],
        nodes[nextIdx],
        penalty);
}

double AStarPathFinder::calculateDirectionPenalty(
    int fromIdx, int toIdx, int goalIdx
) const {
    const auto& nodes = mapInfo_.getNodes();
    
    if (fromIdx < 0 || toIdx < 0 || goalIdx < 0 ||
        fromIdx >= static_cast<int>(nodes.size()) ||
        toIdx >= static_cast<int>(nodes.size()) ||
        goalIdx >= static_cast<int>(nodes.size())) {
        return 0.0;
    }
    
    const Node& from = nodes[fromIdx];
    const Node& to = nodes[toIdx];
    const Node& goal = nodes[goalIdx];
    
    // 计算当前移动方向
    double dx_move = to.x - from.x;
    double dy_move = to.y - from.y;
    
    // 计算目标方向
    double dx_goal = goal.x - from.x;
    double dy_goal = goal.y - from.y;
    
    double len_move = std::sqrt(dx_move*dx_move + dy_move*dy_move);
    double len_goal = std::sqrt(dx_goal*dx_goal + dy_goal*dy_goal);
    
    if (len_move < 1e-6 || len_goal < 1e-6) {
        return 0.0;
    }
    
    // 归一化
    dx_move /= len_move;
    dy_move /= len_move;
    dx_goal /= len_goal;
    dy_goal /= len_goal;
    
    // 计算点积（余弦相似度）
    double dotProduct = dx_move * dx_goal + dy_move * dy_goal;
    
    // 如果方向相反（dotProduct < 0），增加惩罚
    // dotProduct范围: [-1, 1]，-1表示完全相反，1表示完全相同
    if (dotProduct < 0) {
        // 惩罚力度：相反方向惩罚更大
        return len_move * 500.0 * (1.0 - dotProduct);  // 最大1000mm惩罚
    }
    
    return 0.0;
}

std::vector<int> AStarPathFinder::reconstructPath(
    const std::unordered_map<AStarState, AStarState, StateHasher>& cameFrom,
    const AStarState& endState
) const {
    std::vector<int> path;
    AStarState current = endState;

    path.push_back(nodeIndexToId_.at(current.curr));
    while (true) {
        auto it = cameFrom.find(current);
        if (it == cameFrom.end()) {
            break;
        }
        current = it->second;
        path.push_back(nodeIndexToId_.at(current.curr));
    }

    std::reverse(path.begin(), path.end());
    return path;
}

AStarPathFinder::PathResult AStarPathFinder::findPath(
    int startNodeId,
    int endNodeId,
    double turnPenalty,
    const std::unordered_set<int>* bannedNodes,
    const std::set<std::pair<int,int>>* bannedEdges,
    const std::unordered_map<int, double>* nodePenalty,
    double nodePenaltyMm
) {
    PathResult result;

    auto startIt = nodeIdToIndex_.find(startNodeId);
    auto endIt = nodeIdToIndex_.find(endNodeId);

    if (startIt == nodeIdToIndex_.end() || endIt == nodeIdToIndex_.end()) {
        return result;
    }

    int startIdx = startIt->second;
    int endIdx = endIt->second;

    const auto& nodes = mapInfo_.getNodes();
    if (startIdx == endIdx) {
        if (nodes[startIdx].allowPass == 0) {
            return result;
        }
        result.path = {startNodeId};
        result.distance = 0;
        result.found = true;
        return result;
    }
    if (nodes[endIdx].allowPass == 0) {
        return result;
    }

    using State = AStarState;
    State startState{-1, startIdx};

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareNode> openSet;
    std::unordered_set<State, StateHasher> closedSet;
    std::unordered_map<State, double, StateHasher> gScores;
    std::unordered_map<State, State, StateHasher> cameFrom;

    gScores[startState] = 0;
    double h = heuristic(startIdx, endIdx);
    openSet.emplace(-1, startIdx, 0, h);

    const auto& aftNode = mapInfo_.getAftNode();
    const bool usePenalty = (nodePenalty && nodePenaltyMm > 0.0);

    auto is_banned_index = [&](int nodeIdx) -> bool {
        if (!bannedNodes) return false;
        auto idIt = nodeIndexToId_.find(nodeIdx);
        if (idIt == nodeIndexToId_.end()) return false;
        return bannedNodes->count(idIt->second) > 0;
    };
    auto is_banned_edge = [&](int fromIdx, int toIdx) -> bool {
        if (!bannedEdges) return false;
        auto fromIt = nodeIndexToId_.find(fromIdx);
        if (fromIt == nodeIndexToId_.end()) return false;
        auto toIt = nodeIndexToId_.find(toIdx);
        if (toIt == nodeIndexToId_.end()) return false;
        return bannedEdges->count({fromIt->second, toIt->second}) > 0;
    };

    while (!openSet.empty()) {
        AStarNode current = openSet.top();
        openSet.pop();

        State currentState{current.prev, current.curr};
        auto gItCurrent = gScores.find(currentState);
        if (gItCurrent != gScores.end() && current.gScore > gItCurrent->second) {
            continue;
        }

        if (current.curr == endIdx) {
            result.path = reconstructPath(cameFrom, currentState);
            result.distance = current.gScore;
            result.found = true;
            return result;
        }

        if (closedSet.count(currentState)) {
            continue;
        }
        closedSet.insert(currentState);

        // aftNode 使用节点索引作为 key/value
        auto it = aftNode.find(current.curr);
        if (it == aftNode.end()) {
            continue;
        }

        for (int neighborIdx : it->second) {
            if (neighborIdx < 0 || neighborIdx >= static_cast<int>(nodes.size())) {
                continue;
            }
            if (nodes[neighborIdx].allowPass == 0) {
                continue;
            }
            if (is_banned_edge(current.curr, neighborIdx)) {
                continue;
            }
            if (neighborIdx != endIdx && neighborIdx != startIdx && is_banned_index(neighborIdx)) {
                continue;
            }
            State nextState{current.curr, neighborIdx};
            if (closedSet.count(nextState)) {
                continue;
            }

            const Node& currNode = nodes[current.curr];
            const Node& neighNode = nodes[neighborIdx];
            double dx = neighNode.x - currNode.x;
            double dy = neighNode.y - currNode.y;
            double edgeCost = std::sqrt(dx*dx + dy*dy);

            double turnCost = 0.0;
            if (current.prev >= 0) {
                turnCost = calculateTurnPenalty(current.prev, current.curr, neighborIdx, turnPenalty);
            }

            double penaltyCost = 0.0;
            if (usePenalty) {
                auto idIt = nodeIndexToId_.find(neighborIdx);
                if (idIt != nodeIndexToId_.end()) {
                    auto pIt = nodePenalty->find(idIt->second);
                    if (pIt != nodePenalty->end()) {
                        double score = std::clamp(pIt->second, 0.0, 1.0);
                        penaltyCost = nodePenaltyMm * score;
                    }
                }
            }

            double tentativeGScore = current.gScore + edgeCost + turnCost + penaltyCost;

            auto gIt = gScores.find(nextState);
            if (gIt == gScores.end() || tentativeGScore < gIt->second) {
                gScores[nextState] = tentativeGScore;
                cameFrom[nextState] = currentState;

                double fScore = tentativeGScore + heuristic(neighborIdx, endIdx);
                openSet.emplace(current.curr, neighborIdx, tentativeGScore, fScore);
            }
        }
    }

    return result;
}
