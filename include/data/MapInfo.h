#ifndef MAPINFO_H
#define MAPINFO_H

#include <vector>
#include <unordered_map>
#include <cmath>
#include <string>

// 单个节点的信息结构（对应MATLAB的node_res元素）
struct Node {
    int id;                  // 节点ID（原始ID）
    int index;               // 节点索引（从1开始，对应MATLAB的id2index）
    int type;                // 节点类型（-1：无效，7/9：充电站，1/2：有效任务点）
    double x;                // 节点X坐标
    double y;                // 节点Y坐标
    int allowPass;           // 是否允许通行（0/1）
    int allowRot;            // 是否允许原地转向（0/1）
    // 按类型限制：字符串列表，元素可为类型名或类型索引的字符串（如"0","1"）。
    // 为空表示不限定（所有类型均可）。
    std::vector<std::string> allowPassAmrTypeList;
    std::vector<std::string> allowRotAmrTypeList;
    double maxSpeed;         // 节点最大速度（受区域限速影响，对应MATLAB的maxSpeed）
};

// 单个边的信息结构（对应MATLAB的edge_list元素）
struct Edge {
    int startNodeId;         // 起点原始ID
    int endNodeId;           // 终点原始ID
    double weight;           // 边的权重（距离，mm）
    int mode;                // 边的模式（0：跳过，1：单向，2：双向，对应MATLAB的edge.mode）
    int direction;
    double maxSpeed;         // 边上的最大速度限制（mm/s，0表示未指定）
};

// 区域限速信息（对应MATLAB的map_data.area）
struct Area {
    double minX;             // 区域最小X坐标
    double maxX;             // 区域最大X坐标（minX + width）
    double minY;             // 区域最小Y坐标
    double maxY;             // 区域最大Y坐标（minY + height）
    double maxSpeed;         // 区域最大速度限制
};

// 并查集类（实现可达性分析，对应MATLAB的UnionFind）
class UnionFind {
private:
    std::vector<int> parent; // 父节点数组
    std::vector<int> rank;   // 秩（用于按秩合并）
public:
    // 构造函数：初始化n个节点（节点索引从1开始）
    UnionFind(int n);
    // 查找根节点：
    // - 非 const 版本：可做路径压缩（写 parent），仅用于构建阶段
    // - const 版本：只读（不写 parent），避免并发查询时发生数据竞争
    int find(int x);
    int find(int x) const;
    // 合并两个节点（带按秩合并）
    void unite(int x, int y);
    // 判断两个节点是否连通（对应MATLAB的isConnected）
    bool isConnected(int x, int y) const;
};

// 地图信息主类（存储所有地图数据，对应MATLAB的node_res、pre_node等）
class MapInfo {
private:
    std::vector<Node> nodes;                  // 所有有效节点列表（对应MATLAB的node_res）
    std::unordered_map<int, int> id2index;    // 节点ID到索引的映射（对应MATLAB的id2index）
    std::unordered_map<int, std::vector<int>> preNode; // 前驱节点表（key：节点索引，value：前驱索引列表）
    std::unordered_map<int, std::vector<int>> aftNode;  // 后继节点表（key：节点索引，value：后继索引列表）
    std::vector<Edge> edges;                  // 所有有效边列表
    std::unordered_map<size_t, int> edgeIndex;
    size_t getEdgeKey(int startIndex, int endIndex) const;
    std::vector<Area> areas;                  // 区域限速列表
    UnionFind* reachAnalyzer;                 // 可达性分析器（对应MATLAB的reach_analyzer）
    double globalMaxSpeed;                    // 全局最大速度（对应MATLAB的map_data.info.maxSpeed）
    int amrTypeNum;                           // AGV类型数量（对应MATLAB的amr_type_num）

    // 辅助函数：计算向量方向（对应MATLAB的vector_direction）
    int calculateDirection(double dx, double dy);

public:
    // 构造函数：初始化AGV类型数量
    MapInfo(int amrTypeNum);
    // 析构函数：释放可达性分析器内存
    ~MapInfo();

    // 加载节点数据（对应MATLAB的read_node_info核心逻辑）
    void loadNodes(const std::vector<Node>& rawNodes, const std::vector<Area>& mapAreas, double globalMaxSpeed);
    // 加载边数据（对应MATLAB的read_edge_info核心逻辑）
    void loadEdges(const std::vector<Edge>& rawEdges);

    // Getter方法：获取地图数据（避免直接访问私有成员）
    const std::vector<Node>& getNodes() const;
    const std::unordered_map<int, int>& getId2Index() const;
    const std::unordered_map<int, std::vector<int>>& getPreNode() const;
    const std::unordered_map<int, std::vector<int>>& getAftNode() const;
    // 新增方法1：获取所有有效边列表（返回常量引用，避免拷贝）
    const std::vector<Edge>& getEdges() const;
    // 新增方法2：根据节点索引获取节点对象（返回常量引用，高效访问）
    // 若索引无效，抛出异常
    const Node& getNode(int nodeIndex) const;
    // 新增方法2b：根据节点ID获取节点对象（外部统一用ID查询，内部自行映射）
    // 若ID无效，抛出异常
    const Node& getNodeById(int nodeId) const;
    // 新增方法3：根据起点和终点索引获取边对象（返回常量指针，找不到返回nullptr）
    const Edge* getEdge(int startIndex, int endIndex) const;
    // 获取边的限速（优先使用边级限制，不存在则回退到节点/全局限速）
    double getEdgeMaxSpeed(int startNodeId, int endNodeId) const;
    const UnionFind* getReachAnalyzer() const;
    double getGlobalMaxSpeed() const;
    int getAmrTypeNum() const;
    // 按节点ID快速判断连通（用于预过滤，不投Dijkstra）
    bool isConnectedById(int startNodeId, int endNodeId) const;
    // 获取充电站列表（对应MATLAB的charge_station_list）
    std::vector<int> getChargeStationList() const;
    // 获取有效任务节点列表（type=1或2，对应MATLAB的valid_point_id）
    std::vector<int> getValidTaskNodeList(int keyNodeIndex) const;
    // 在MapInfo类的public方法声明中添加
    /**
     * @brief 从所有节点中查找距离(x, y)最近的节点ID
     * @param x 目标x坐标
     * @param y 目标y坐标
     * @return 最近节点的ID，若节点列表为空则抛出异常
     */
    int findNearestNodeId(double x, double y) const;

    /**
     * @brief 从参考节点的前驱、后继及自身中查找距离(x, y)最近的节点ID
     * @param x 目标x坐标
     * @param y 目标y坐标
     * @param refNodeId 参考节点ID
     * @return 最近节点的ID，若参考节点无效则抛出异常
     */
    int findNearestNodeIdFromNeighbors(double x, double y, int refNodeId) const;

};

#endif // MAPINFO_H
