#ifndef STATIC_PATH_TABLE_H
#define STATIC_PATH_TABLE_H

#include <vector>
#include <map>
#include <optional>
#include <string>
#include <set>
#include <cstdint>

// 前向声明
class MapInfo;

/**
 * @brief 静态路径表 - 工业级实现
 * 
 * 设计目标：
 * 1. 离线预计算所有任务点对之间的Top-3路径
 * 2. 处理不可达路径（约28.5%）
 * 3. 考虑转向惩罚、速度限制等实际因素
 * 4. 支持动态路径选择（运行时根据拥堵状态）
 * 5. 对标海康RCS-2000的路径规划能力
 */
class StaticPathTable {
public:
    // ==================== 数据结构定义 ====================
    
    /**
     * @brief 路径候选（单条路径）
     */
    struct PathCandidate {
        std::vector<int> fullPath;            // 完整路径（节点ID序列）
        std::vector<int> keyNodes;            // 关键节点（起点、转弯点、终点，节点ID）
        
        // 成本信息
        double baseDistance;                  // 基础距离（mm）
        double baseCost;                      // 基础成本（考虑转向）
        double estimatedTime;                 // 预计时间（秒）
        
        // 路径特征
        int turnCount;                        // 转弯次数
        int segmentCount;                     // 路段数量
        double avgSegmentLength;              // 平均路段长度
        
        // 路径质量标签
        enum Quality {
            OPTIMAL = 0,    // 最优（成本最低）
            FAST = 1,       // 快速（转弯少）
            SAFE = 2        // 安全（拥堵少）
        } quality;
        
        // 高级特征（用于动态选择）
        std::vector<int> bottleneckNodes;     // 瓶颈节点（可能拥堵）
        std::vector<int> criticalTurns;       // 关键转弯点
        bool hasNarrowSection;                // 是否有窄通道
        bool crossMainTraffic;                // 是否穿越主干道
        int maxConcurrentAgvs;                // 最大并发AGV数
        
        PathCandidate();
    };
    
    /**
     * @brief 路径条目（一对起终点的所有路径）
     */
    struct PathEntry {
        int startNode;                        // 起点节点ID
        int endNode;                          // 终点节点ID
        bool reachable;                       // 是否可达
        
        std::vector<PathCandidate> candidates; // Top-3路径
        
        // 元数据
        long long generateTime;               // 生成时间戳
        int queryCount;                       // 查询次数（运行时统计）
        
        PathEntry();
    };
    
    /**
     * @brief 动态上下文（运行时信息）
     */
    struct DynamicContext {
        // 实时交通状态
        std::map<int, double> nodeCongestion;  // 节点拥堵度 [0,1]
        std::map<int, double> edgeCongestion;  // 边拥堵度 [0,1]
        
        // 临时障碍
        std::set<int> blockedNodes;            // 临时障碍节点
        std::set<std::pair<int,int>> blockedEdges; // 临时障碍边
        
        // 偏好设置
        bool preferLessTurns;                  // 偏好少转弯
        bool avoidCongestion;                  // 避免拥堵
        bool preferShortest;                   // 偏好最短距离
        double turnPenaltyWeight;              // 转弯惩罚权重
        
        DynamicContext();
        double getNodeCongestion(int nodeId) const;
    };
    
    /**
     * @brief 生成配置
     */
    struct GenerateConfig {
        int topK;                              // 生成Top-K路径（默认3）
        double turnPenaltyMm;                  // 转向惩罚（mm，默认3000）
        double defaultSpeed;                   // 默认速度（mm/s，默认1200）
        int numThreads;                        // 线程数（默认8）
        bool enableCompression;                // 启用压缩（默认true）
        bool verboseOutput;                    // 详细输出（默认true）
        
        GenerateConfig();
    };
    
    // ==================== 公共接口 ====================
    
    StaticPathTable();
    ~StaticPathTable() = default;
    
    /**
     * @brief 生成静态路径表（离线）
     * @param mapInfo 地图信息
     * @param config 生成配置
     * @return 是否成功
     */
    bool generate(const MapInfo& mapInfo, const GenerateConfig& config = GenerateConfig());
    
    /**
     * @brief 查询路径（运行时）
     * @param startNode 起点索引
     * @param endNode 终点索引
     * @param context 动态上下文
     * @return 最优路径候选（如果不可达返回nullopt）
     */
    std::optional<PathCandidate> query(
        int startNode, 
        int endNode,
        const DynamicContext& context = DynamicContext()
    ) const;
    
    /**
     * @brief 保存到文件
     * @param filePath 文件路径
     * @return 是否成功
     */
    bool saveToFile(const std::string& filePath) const;
    
    /**
     * @brief 从文件加载
     * @param filePath 文件路径
     * @return 是否成功
     */
    bool loadFromFile(const std::string& filePath);
    
    /**
     * @brief 获取统计信息
     */
    struct Statistics {
        size_t totalPairs;
        size_t reachablePairs;
        size_t unreachablePairs;
        double reachabilityRate;
        size_t totalCandidates;
        double avgCandidatesPerPair;
        size_t fileSizeBytes;
    };
    
    Statistics getStatistics() const;
    
    /**
     * @brief 验证路径表完整性
     */
    bool validate() const;
    
private:
    // ==================== 内部数据结构 ====================
    
    struct PathKey {
        int startNode;
        int endNode;
        
        bool operator<(const PathKey& other) const {
            if (startNode != other.startNode) return startNode < other.startNode;
            return endNode < other.endNode;
        }
    };
    
    std::map<PathKey, PathEntry> pathTable_;
    Statistics stats_;
    
    // ==================== 内部方法 ====================
    
    // 节点提取
    std::vector<int> extractTaskNodes(const MapInfo& mapInfo);
    std::vector<int> extractChargeNodes(const MapInfo& mapInfo);
    
    // Top-K路径计算
    std::vector<PathCandidate> computeTopKPaths(
        int startIdx,
        int endIdx,
        const MapInfo& mapInfo,
        const GenerateConfig& config
    );
    
    // 路径分析
    void analyzePathFeatures(
        PathCandidate& candidate,
        const MapInfo& mapInfo
    );
    
    int countTurns(const std::vector<int>& path, const MapInfo& mapInfo);
    std::vector<int> extractKeyNodes(const std::vector<int>& path, const MapInfo& mapInfo);
    std::vector<int> findBottlenecks(const std::vector<int>& path, const MapInfo& mapInfo);
    double calculatePathDistance(const std::vector<int>& path, const MapInfo& mapInfo);
    double calculatePathCost(const std::vector<int>& path, const MapInfo& mapInfo, double turnPenalty);
    
    // 动态选择
    std::optional<PathCandidate> selectBestCandidate(
        const std::vector<PathCandidate>& candidates,
        const DynamicContext& context
    ) const;
    
    double scorePath(const PathCandidate& candidate, const DynamicContext& context) const;
};

#endif // STATIC_PATH_TABLE_H
