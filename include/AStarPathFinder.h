#ifndef ASTAR_PATH_FINDER_H
#define ASTAR_PATH_FINDER_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <optional>
#include <limits>
#include <functional>
#include "common/PathPlanningConstants.h"
#include "data/MapInfo.h"

/**
 * @brief A*路径查找器（返回完整路径）
 * 
 * 专为GlobalPathPlanner设计，返回完整的节点序列
 */
class AStarPathFinder {
public:
    using DynamicNodePenaltyFn = std::function<double(int prevNodeId,
                                                      int nodeId,
                                                      double arrivalCostMm)>;

    struct PathResult {
        std::vector<int> path;      // 完整路径（节点ID序列）
        double distance;            // 路径距离（mm）
        bool found;                 // 是否找到路径
        
        PathResult() : distance(0), found(false) {}
    };
    
    explicit AStarPathFinder(const MapInfo& mapInfo);
    
    /**
     * @brief A*搜索路径
     * @param startNodeId 起点节点ID
     * @param endNodeId 终点节点ID
     * @param turnPenalty 转弯惩罚（mm）
     * @param bannedNodes 可选：禁止经过的节点ID集合（nullptr表示不限制）
     * @param bannedEdges 可选：禁止经过的边集合（以节点ID对表示的有向边，nullptr表示不限制）
     * @return 路径结果
     */
    PathResult findPath(int startNodeId,
                        int endNodeId,
                        double turnPenalty = PathPlanningConstants::resolveTurnPenaltyMm(),
                        const std::unordered_set<int>* bannedNodes = nullptr,
                        const std::set<std::pair<int,int>>* bannedEdges = nullptr,
                        const std::unordered_map<int, double>* nodePenalty = nullptr,
                        double nodePenaltyMm = 0.0,
                        DynamicNodePenaltyFn dynamicNodePenaltyFn = {});
    
private:
    const MapInfo& mapInfo_;
    
    // 节点索引映射
    std::unordered_map<int, int> nodeIdToIndex_;
    std::unordered_map<int, int> nodeIndexToId_;
    
    // A*搜索状态 (prev, curr)
    struct AStarState {
        int prev;
        int curr;

        bool operator==(const AStarState& other) const {
            return prev == other.prev && curr == other.curr;
        }
    };

    struct StateHasher {
        size_t operator()(const AStarState& s) const {
            size_t h1 = std::hash<int>{}(s.prev);
            size_t h2 = std::hash<int>{}(s.curr);
            return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
        }
    };

    // A*搜索节点
    struct AStarNode {
        int prev;
        int curr;
        double travelCost;  // 到当前节点的名义行驶代价（不含软惩罚）
        double scoreCost;   // 搜索评分代价（含软惩罚）
        double fScore;      // scoreCost + 启发式估计

        AStarNode(int p, int c, double travel, double score, double f)
            : prev(p), curr(c), travelCost(travel), scoreCost(score), fScore(f) {}
    };
    
    // 优先队列比较器
    struct CompareNode {
        bool operator()(const AStarNode& a, const AStarNode& b) const {
            return a.fScore > b.fScore;  // 小顶堆
        }
    };
    
    // 辅助函数
    void buildNodeMapping();
    double heuristic(int fromIdx, int toIdx) const;
    double calculateTurnPenalty(int prevIdx, int currIdx, int nextIdx, double penalty) const;
    double calculateDirectionPenalty(int fromIdx, int toIdx, int goalIdx) const;
    std::vector<int> reconstructPath(
        const std::unordered_map<AStarState, AStarState, StateHasher>& cameFrom,
        const AStarState& endState
    ) const;
};

#endif // ASTAR_PATH_FINDER_H
