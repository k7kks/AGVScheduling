#ifndef SHORTEST_PATH_UPDATER_H
#define SHORTEST_PATH_UPDATER_H

#include <vector>
#include <unordered_map>
#include <mutex>
#include <queue>  // 新增：引入std::priority_queue
#include <functional>  // 新增：引入std::greater（用于比较器）
#include "data/MapInfo.h"
#include <atomic>  // 需引入原子类型头文件
#include <condition_variable>
#include <thread>

// 距离矩阵状态（对应MATLAB的-1/-2/>0）
enum class DistanceState {
    UNCALCULATED = -1,  // 未计算
    UNREACHABLE = -2,   // 不可达
    VALID = 0           // 有效（值>0）
};

// 优先队列元素：<总代价fScore, AMR类型, 节点索引>（与原逻辑一致）
struct QueueElement {
    double fScore;
    int amrType;
    int nodeIndex;
};

// 自定义比较器：使std::priority_queue成为小顶堆（按fScore升序）
struct QueueComparator {
    bool operator()(const QueueElement& a, const QueueElement& b) const {
        // 大顶堆默认用a < b，此处反转为a.fScore > b.fScore，实现小顶堆
        return a.fScore > b.fScore;
    }
};

class ShortestPathUpdater {
private:
    // 需标记为 mutable 的成员
    mutable std::vector<std::vector<std::vector<double>>> distanceMatrix;
    mutable bool matrixInitialized;
    mutable std::vector<std::tuple<int, int, int>> queryQueue;
    mutable int activeWorkers;
    mutable std::mutex queueMutex;
    mutable std::condition_variable queueCv;
    mutable std::mutex matrixMutex;
    // 只读成员（无需 mutable）
    const MapInfo& mapInfo;
    double globalMaxSpeed;
    double directionPenalty;
    int amrTypeNum_ = 0; // 存储 initializeMatrix 传入的类型数
    const int maxWorkerNum;
    mutable std::atomic<bool> shouldExit;
    mutable std::vector<std::thread> workers;


    // 辅助方法（保持原实现，无需修改）
    double minkowskiHeuristic(int startIdx, int endIdx, double p = 1.0) const;
    bool isPathValid(int startIdx, int endIdx) const;
    void shortestPathFinder(int startIdx, int endIdx, int amrType);
    void workerFunction();

public:
    ShortestPathUpdater(const MapInfo& mapInfo, double directionPenalty, int maxWorkerNum = 4);
    ~ShortestPathUpdater();

    void requestExit() const {
        shouldExit = true;  // 原子操作，安全通知所有线程
    }
    void initializeMatrix(int amrTypeNum);
    double query(int startIdx, int endIdx, int amrType) const;  // 保持 const 声明
    // 新增：按节点ID查询（内部自行映射到索引），返回当前矩阵状态值：
    // UNCALCULATED(-1) / UNREACHABLE(-2) / 有效时间(>0)
    double queryByNodeId(int startNodeId, int endNodeId, int amrType) const;
    void batchQueryByNodeId(int startNodeId, const std::vector<int>& targetNodeIds, int amrType) const;
    void setDirectionPenalty(double penalty);
    const std::vector<std::vector<std::vector<double>>>& getDistanceMatrix() const;
    // 新增：获取当前活跃的工作线程数（线程安全）
    int getActiveWorkers() const {
        std::lock_guard<std::mutex> lock(queueMutex);  // 加锁保护，避免多线程竞争
        return activeWorkers;
    }

};

#endif // SHORTEST_PATH_UPDATER_H
