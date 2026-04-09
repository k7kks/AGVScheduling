# AGV路径规划三阶段分析文档

## 背景说明

在AGV集群调度系统中，路径规划分为三个阶段：**Fast**、**Major**、**Super**。

**核心约束**：一旦向机器人发送轨迹（Trail），机器人**必定会执行**该轨迹。这意味着：
- 发送轨迹 `1→2→3→4` 后，机器人会执行这个轨迹
- 再发送轨迹 `1→2→3→5` 是**违反commit约束**的
- 因为机器人已经承诺执行第一个轨迹，不能中途改变

---

## 三个阶段定义

### 1. Fast阶段（快速规划）

**定义**：基于当前已知信息的快速路径规划

**特点**：
- 规划时间短（<100ms）
- 基于静态地图和当前AGV位置
- 不考虑其他AGV的动态位置
- 可能不是最优路径

**输出**：初步路径（未commit）

**问题**：
- 可能与其他AGV冲突
- 可能被后续阶段推翻

---

### 2. Major阶段（主要规划）

**定义**：考虑冲突检测和预约的路径规划

**特点**：
- 规划时间中等（100-500ms）
- 基于节点预约表（NodeReservationTable）
- 检查路径上的节点是否被其他AGV预约
- 如果冲突，尝试重新规划

**输出**：冲突检测后的路径（部分commit）

**关键操作**：
```
1. 规划初步路径（Fast阶段的结果）
2. 检查路径上的每个节点是否被预约
3. 如果节点被其他AGV预约：
   - 尝试绕路
   - 或等待预约释放
   - 或触发重规划
4. 返回无冲突的路径
```

**问题**：
- 预约窗口有限（TRAIL_MAX_POINTS=4）
- 后续节点可能无保护
- 其他AGV可能抢先预约后续节点

---

### 3. Super阶段（超级规划）

**定义**：全局优化和死锁检测的路径规划

**特点**：
- 规划时间长（500ms-2s）
- 考虑全局AGV分布
- 检测死锁情况
- 可能调整多个AGV的路径
- 考虑优先级和任务紧急度

**输出**：全局优化的路径（完全commit）

**关键操作**：
```
1. 收集所有AGV的当前路径
2. 构建冲突图
3. 检测死锁（环形等待）
4. 全局优化：
   - 调整低优先级AGV的路径
   - 为高优先级AGV让路
   - 最小化总体延迟
5. 返回优化后的路径
```

**问题**：
- 规划时间长，可能错过实时性
- 全局优化复杂度高
- 可能导致某些AGV长期等待

---

## 核心问题分析

### 问题1：Commit约束违反

**场景**：
```
时间轴：
t=0s:   Fast阶段规划路径：1→2→3→4
        发送给机器人

t=0.1s: Major阶段检测到冲突
        重新规划路径：1→2→3→5
        再次发送给机器人

问题：机器人已经收到第一个轨迹，正在执行
      现在收到第二个轨迹，应该执行哪个？

结果：
- 如果执行第一个：浪费了重规划的努力
- 如果执行第二个：违反了commit约束
- 如果都执行：路径混乱
```

**根本原因**：
- 三个阶段的规划结果可能相互矛盾
- 没有明确的commit时机
- 没有版本控制机制

**影响**：
- 机器人行为不可预测
- 系统状态不一致
- 可能导致碰撞

---

### 问题2：预约窗口不足导致后续节点无保护

**场景**：
```
配置：TRAIL_MAX_POINTS=4（预约未来4个节点）

时间轴：
t=0s:   AGV_A Fast阶段规划路径：1→2→3→4→5→6→7→8
        预约节点：2,3,4,5（未来4个节点）
        发送轨迹：1→2→3→4

t=0.5s: AGV_B Fast阶段规划路径：8→7→6→5
        检查预约：节点8、7、6、5
        节点8、7、6都未被预约！（超出AGV_A的预约窗口）
        AGV_B 预约成功
        发送轨迹：8→7→6→5

t=1s:   AGV_A Major阶段规划路径：1→2→3→4→5→6→7→8
        检查预约：节点5、6、7、8
        节点6、7、8被AGV_B预约！
        冲突！

t=1.5s: AGV_A Super阶段全局优化
        但已经太晚了，两个AGV都已经commit
        无法改变

结果：两个AGV在节点6-7之间相遇 → 碰撞
```

**根本原因**：
- 预约窗口固定为4个节点
- 快速AGV的后续节点无保护
- 其他AGV可以抢先预约

**影响**：
- 路径冲突
- 系统无法避免碰撞

---

### 问题3：三个阶段的规划结果不一致

**场景**：
```
时间轴：
t=0s:   Fast阶段：规划路径 1→2→3→4（无冲突检测）

t=0.1s: Major阶段：检测到节点3被预约
        重新规划路径 1→2→5→4（绕路）

t=0.5s: Super阶段：全局优化
        发现AGV_B的优先级更高
        重新规划路径 1→6→7→4（让路）

问题：三个阶段的路径完全不同！
      1→2→3→4 vs 1→2→5→4 vs 1→6→7→4

      机器人应该执行哪个？
      如果都发送，机器人会混乱
      如果只发送最后一个，前面的规划浪费了
```

**根本原因**：
- 没有明确的"最终规划"
- 三个阶段独立运行，无协调
- 没有版本控制

**影响**：
- 规划结果不确定
- 机器人行为不可预测

---

### 问题4：Commit时机不明确

**场景**：
```
问题：什么时候应该commit路径？

选项1：Fast阶段后立即commit
- 优点：快速响应
- 缺点：可能有冲突

选项2：Major阶段后commit
- 优点：冲突检测
- 缺点：可能被Super阶段推翻

选项3：Super阶段后commit
- 优点：全局优化
- 缺点：延迟太长（500ms-2s）

选项4：不commit，持续更新
- 优点：灵活
- 缺点：违反约束，机器人混乱
```

**根本原因**：
- 没有明确的commit策略
- 三个阶段的目标不一致

**影响**：
- 系统设计模糊
- 实现困难

---

### 问题5：预约表的TTL过期导致冲突

**场景**：
```
配置：RESERVE_TTL_MS=15000ms（15秒TTL）

时间轴：
t=0s:   AGV_A Fast阶段规划路径：1→2→3→4→5
        预约节点：2,3,4,5（TTL=15秒）
        发送轨迹：1→2→3→4

t=5s:   AGV_A 在节点3停留（故障、等待）

t=15s:  节点2的预约TTL过期 → 自动释放
        节点3的预约TTL过期 → 自动释放
        节点4的预约TTL过期 → 自动释放

t=15.5s: AGV_B Fast阶段规划路径：1→2→3→4
        检查预约：节点2、3、4都未被预约！
        AGV_B 预约成功
        发送轨迹：1→2→3→4

t=16s:  AGV_A 仍在节点3（故障未排除）
        AGV_B 到达节点3
        碰撞！❌
```

**根本原因**：
- TTL是固定的15秒
- 如果AGV停留超过15秒，预约失效
- 其他AGV可以进入

**影响**：
- 故障AGV导致碰撞
- 系统不可靠

---

## 可行的解决方案

### 方案1：明确的Commit策略

**核心思想**：定义清晰的commit时机和版本控制

**实现**：

```cpp
// 路径版本控制
struct PathVersion {
    int version;                    // 版本号（递增）
    std::string stage;              // 阶段：FAST/MAJOR/SUPER
    std::vector<int> path;          // 路径
    std::chrono::steady_clock::time_point timestamp;
    bool committed = false;         // 是否已commit
};

class PathVersionManager {
private:
    std::unordered_map<std::string, PathVersion> currentPaths_;  // AGV ID -> 当前路径
    std::unordered_map<std::string, PathVersion> committedPaths_; // AGV ID -> 已commit路径
    std::mutex mutex_;

public:
    // ✅ 提议新路径（不commit）
    bool proposeNewPath(const std::string& agvId,
                       const std::string& stage,
                       const std::vector<int>& path) {
        std::lock_guard<std::mutex> lk(mutex_);

        auto it = currentPaths_.find(agvId);
        if (it != currentPaths_.end()) {
            // 检查是否与已commit路径冲突
            if (it->second.committed) {
                // 已commit的路径不能改变
                if (pathsConflict(it->second.path, path)) {
                    return false;  // 拒绝提议
                }
            }
        }

        // 接受提议
        PathVersion newVersion;
        newVersion.version = (it != currentPaths_.end()) ? it->second.version + 1 : 1;
        newVersion.stage = stage;
        newVersion.path = path;
        newVersion.timestamp = std::chrono::steady_clock::now();
        newVersion.committed = false;

        currentPaths_[agvId] = newVersion;
        return true;
    }

    // ✅ Commit路径（不可逆）
    bool commitPath(const std::string& agvId) {
        std::lock_guard<std::mutex> lk(mutex_);

        auto it = currentPaths_.find(agvId);
        if (it == currentPaths_.end()) {
            return false;
        }

        // 标记为已commit
        it->second.committed = true;
        committedPaths_[agvId] = it->second;

        // 发送给机器人
        sendPathToRobot(agvId, it->second.path, it->second.version);

        return true;
    }

    // ✅ 获取当前路径
    std::optional<PathVersion> getCurrentPath(const std::string& agvId) {
        std::lock_guard<std::mutex> lk(mutex_);

        auto it = currentPaths_.find(agvId);
        if (it == currentPaths_.end()) {
            return std::nullopt;
        }

        return it->second;
    }

    // ✅ 检查路径是否已commit
    bool isPathCommitted(const std::string& agvId) {
        std::lock_guard<std::mutex> lk(mutex_);

        auto it = committedPaths_.find(agvId);
        return it != committedPaths_.end();
    }

private:
    bool pathsConflict(const std::vector<int>& path1,
                      const std::vector<int>& path2) {
        // 检查两个路径是否有冲突
        // 如果已commit的路径是 1→2→3→4
        // 新路径不能是 1→2→3→5（改变了已commit的部分）

        int minLen = std::min(path1.size(), path2.size());
        for (int i = 0; i < minLen; ++i) {
            if (path1[i] != path2[i]) {
                return true;  // 冲突
            }
        }
        return false;
    }

    void sendPathToRobot(const std::string& agvId,
                        const std::vector<int>& path,
                        int version) {
        // 发送路径给机器人
        // 包含版本号，机器人可以验证
    }
};
```

**Commit策略**：
```
1. Fast阶段：提议路径（不commit）
2. Major阶段：
   - 如果无冲突，commit路径
   - 如果有冲突，提议新路径（不commit）
3. Super阶段：
   - 如果已commit，不能改变
   - 如果未commit，可以优化后commit
```

**优点**：
- 清晰的commit时机
- 版本控制，可追踪
- 已commit的路径不可改变

**缺点**：
- 实现复杂
- 可能限制优化空间

---

### 方案2：动态预约窗口

**核心思想**：根据AGV速度和TTL动态调整预约窗口

**实现**：

```cpp
class DynamicReservationWindow {
private:
    int baseWindowSize_ = 4;        // 基础窗口大小
    int maxWindowSize_ = 20;        // 最大窗口大小
    int nodeDistance_ = 1000;       // 节点间距（mm）
    int reserveTtlMs_ = 15000;      // 预约TTL（ms）

public:
    // ✅ 计算动态窗口大小
    int calculateWindowSize(double agvSpeed) {
        // 窗口大小 = ceil(agvSpeed * reserveTtlMs / nodeDistance)
        // 确保预约覆盖AGV在TTL内能到达的所有节点

        if (agvSpeed <= 0) {
            return baseWindowSize_;
        }

        int dynamicSize = static_cast<int>(
            std::ceil(agvSpeed * reserveTtlMs_ / nodeDistance_)
        );

        return std::min(dynamicSize, maxWindowSize_);
    }

    // ✅ 预约完整路径
    bool reserveFullPath(const std::string& agvId,
                        const std::vector<int>& fullPath,
                        double agvSpeed,
                        NodeReservationTable& nodeTable) {
        int windowSize = calculateWindowSize(agvSpeed);

        // 预约完整路径（不仅仅是窗口）
        for (size_t i = 0; i < fullPath.size(); ++i) {
            int nodeId = fullPath[i];

            // 计算该节点的TTL
            // 节点越远，TTL越长
            double distanceFromStart = i * nodeDistance_;
            double timeToReachNode = distanceFromStart / agvSpeed;
            int nodeTtl = static_cast<int>(timeToReachNode + 5000);  // +5秒缓冲

            if (!nodeTable.tryReserve(nodeId, agvId,
                                     NodeReservationTable::HoldReason::RESERVED_PATH,
                                     "path_node_" + std::to_string(i),
                                     std::chrono::milliseconds(nodeTtl))) {
                // 预约失败，回滚
                nodeTable.releaseAllByOwner(agvId);
                return false;
            }
        }

        return true;
    }
};
```

**优点**：
- 自适应预约窗口
- 快速AGV获得更大的保护范围
- 减少冲突

**缺点**：
- 需要知道AGV速度
- 计算复杂度增加

---

### 方案3：分层规划策略

**核心思想**：三个阶段有明确的职责和约束

**实现**：

```cpp
class HierarchicalPathPlanning {
private:
    enum class PlanningStage {
        FAST,   // 快速规划：无冲突检测
        MAJOR,  // 主要规划：冲突检测和绕路
        SUPER   // 超级规划：全局优化（仅在未commit时）
    };

    PathVersionManager versionManager_;

public:
    // ✅ Fast阶段：快速规划
    bool fastPlanning(const std::string& agvId,
                     int startNode,
                     int endNode,
                     GlobalPathPlanner& planner) {
        auto pathResult = planner.planGlobalPath(startNode, endNode);

        if (!pathResult.reachable) {
            return false;
        }

        // 提议路径（不commit）
        return versionManager_.proposeNewPath(agvId, "FAST", pathResult.path);
    }

    // ✅ Major阶段：冲突检测
    bool majorPlanning(const std::string& agvId,
                      const std::vector<Amr>& amrList,
                      const NodeReservationTable& nodeTable,
                      GlobalPathPlanner& planner) {
        auto currentPath = versionManager_.getCurrentPath(agvId);
        if (!currentPath) {
            return false;
        }

        // 检查是否已commit
        if (currentPath->committed) {
            // 已commit，不能改变
            return true;
        }

        // 检查冲突
        bool hasConflict = false;
        for (int nodeId : currentPath->path) {
            auto entry = nodeTable.get(nodeId);
            if (entry && entry->ownerAgvId != agvId) {
                hasConflict = true;
                break;
            }
        }

        if (!hasConflict) {
            // 无冲突，commit路径
            return versionManager_.commitPath(agvId);
        }

        // 有冲突，尝试绕路
        // 重新规划...
        // 提议新路径（不commit）

        return true;
    }

    // ✅ Super阶段：全局优化（仅在未commit时）
    bool superPlanning(const std::string& agvId,
                      const std::vector<Amr>& amrList,
                      const std::vector<Task>& taskList,
                      GlobalPathPlanner& planner) {
        auto currentPath = versionManager_.getCurrentPath(agvId);
        if (!currentPath) {
            return false;
        }

        // 检查是否已commit
        if (currentPath->committed) {
            // 已commit，不能改变
            return true;
        }

        // 全局优化...
        // 提议新路径（不commit）

        return true;
    }
};
```

**优点**：
- 清晰的阶段职责
- 已commit的路径不可改变
- 灵活的优化空间

**缺点**：
- 实现复杂
- 需要协调三个阶段

---

### 方案4：状态同步机制

**核心思想**：预约表与AGV实际位置保持同步

**实现**：

```cpp
class SyncedReservationManager {
private:
    NodeReservationTable nodeTable_;
    std::unordered_map<std::string, std::vector<int>> agvPaths_;  // AGV完整路径
    std::unordered_map<std::string, int> agvCurrentIndex_;        // AGV在路径中的索引
    std::mutex mutex_;

public:
    // ✅ 更新AGV位置
    void updateAgvPosition(const std::string& agvId, int actualNodeId) {
        std::lock_guard<std::mutex> lk(mutex_);

        auto pathIt = agvPaths_.find(agvId);
        if (pathIt == agvPaths_.end()) {
            return;
        }

        const auto& path = pathIt->second;

        // 找到AGV在路径中的实际位置
        int actualIndex = -1;
        for (size_t i = 0; i < path.size(); ++i) {
            if (path[i] == actualNodeId) {
                actualIndex = i;
                break;
            }
        }

        if (actualIndex < 0) {
            // AGV不在预期路径上，释放所有预约
            nodeTable_.releaseAllByOwner(agvId);
            agvPaths_.erase(pathIt);
            return;
        }

        auto indexIt = agvCurrentIndex_.find(agvId);
        int oldIndex = (indexIt != agvCurrentIndex_.end()) ? indexIt->second : -1;

        // 释放已经过的节点的预约
        if (oldIndex >= 0) {
            for (int i = oldIndex; i < actualIndex; ++i) {
                nodeTable_.release(path[i], agvId);
            }
        }

        // 更新当前索引
        agvCurrentIndex_[agvId] = actualIndex;
    }

    // ✅ 预约路径
    bool reservePath(const std::string& agvId,
                    const std::vector<int>& fullPath,
                    double agvSpeed) {
        std::lock_guard<std::mutex> lk(mutex_);

        // 预约完整路径
        for (size_t i = 0; i < fullPath.size(); ++i) {
            int nodeId = fullPath[i];

            // 计算该节点的TTL
            double distanceFromStart = i * 1000;  // 假设节点间距1000mm
            double timeToReachNode = distanceFromStart / agvSpeed;
            int nodeTtl = static_cast<int>(timeToReachNode + 5000);

            if (!nodeTable_.tryReserve(nodeId, agvId,
                                      NodeReservationTable::HoldReason::RESERVED_PATH,
                                      "path_node_" + std::to_string(i),
                                      std::chrono::milliseconds(nodeTtl))) {
                // 预约失败，回滚
                nodeTable_.releaseAllByOwner(agvId);
                return false;
            }
        }

        agvPaths_[agvId] = fullPath;
        agvCurrentIndex_[agvId] = 0;
        return true;
    }
};
```

**优点**：
- 预约表与实际位置同步
- 减少TTL过期导致的冲突
- 故障AGV不会导致碰撞

**缺点**：
- 需要频繁更新AGV位置
- 网络延迟可能导致不同步

---

## 建议实施顺序

1. **第一步**：实现方案1（明确的Commit策略）
   - 定义清晰的commit时机
   - 版本控制
   - 预防commit约束违反

2. **第二步**：实现方案4（状态同步机制）
   - 保持预约表与实际位置同步
   - 减少TTL过期导致的冲突

3. **第三步**：实现方案2（动态预约窗口）
   - 根据AGV速度调整预约窗口
   - 减少快速AGV的冲突

4. **第四步**：实现方案3（分层规划策略）
   - 明确三个阶段的职责
   - 协调三个阶段的规划

---

## 总结

| 问题 | 根本原因 | 解决方案 |
|------|--------|--------|
| Commit约束违反 | 无版本控制 | 方案1：明确的Commit策略 |
| 预约窗口不足 | 窗口固定 | 方案2：动态预约窗口 |
| 规划结果不一致 | 三阶段无协调 | 方案3：分层规划策略 |
| TTL过期导致冲突 | 状态不同步 | 方案4：状态同步机制 |

**优先级**：方案1 > 方案4 > 方案2 > 方案3
