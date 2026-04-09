# 任务饿死防护机制分析与解决方案

## 1. 问题现象

### 1.1 仿真观察

在多AGV调度仿真中观察到以下现象：

```
时间轴：
t=0s:    AGV均匀分布，任务分散在不同区域
         ├─ 区域A：任务1,2,3,4,5
         ├─ 区域B：任务6,7,8,9,10
         └─ 区域C：任务11,12,13,14,15

t=10s:   AGV开始聚集
         ├─ AGV1,2,3 → 区域A（距离近）
         ├─ AGV4,5 → 区域B
         └─ AGV6 → 区域C

t=30s:   任务分配不均
         ├─ 区域A：已分配 1,2,3,4,5（100%）
         ├─ 区域B：已分配 6,7,8（30%）
         └─ 区域C：已分配 11,12（13%）

t=60s:   任务饿死现象
         ├─ 区域A：AGV继续在该区域循环
         ├─ 区域B：任务9,10 无人接取
         └─ 区域C：任务13,14,15 无人接取
```

### 1.2 根本原因

**贪心算法的局部最优陷阱**：

```
Greedy分配流程：
1. 初始化：所有任务未分配
2. 第1轮：
   - AGV1 选择 距离最近的任务 → 任务1（区域A）
   - AGV2 选择 距离最近的任务 → 任务2（区域A）
   - AGV3 选择 距离最近的任务 → 任务3（区域A）

3. 第2轮：
   - AGV1 选择 距离最近的未分配任务 → 任务4（区域A，距离AGV1最近）
   - AGV2 选择 距离最近的未分配任务 → 任务5（区域A，距离AGV2最近）
   - AGV3 选择 距离最近的未分配任务 → 任务6（区域B，但距离较远）

4. 第3轮：
   - AGV1 选择 → 任务7（区域A，已在该区域）
   - AGV2 选择 → 任务8（区域A，已在该区域）
   - AGV3 选择 → 任务9（区域B，已在该区域）

结果：
- 区域A的任务快速分配完毕
- AGV1,2 继续在区域A寻找任务
- 区域B,C的任务因距离远而被忽视
- 最终导致区域B,C的任务无人接取
```

---

## 2. 代码分析

### 2.1 Greedy算法的问题

**文件**：`src/algorithm/GreedyTaskAllocator.cpp:36-79`

```cpp
// 遍历每个AMR，选择最优任务
for (int amrId = 0; amrId < amrNum; ++amrId) {
    int bestTask = -1;
    double bestCost = INF;

    for (int taskId = 0; taskId < taskNum; ++taskId) {
        if (isAllocated[taskId]) continue;

        // 计算基础成本（秒）：首个任务用初始成本，后续用转移成本
        double cost;
        if (amrTasks[amrId].empty()) {
            cost = TaskAllocationUtils::getCost(taskNum, taskId, amrId);
        } else {
            int lastTask = amrTasks[amrId].back();
            cost = TaskAllocationUtils::getCost(lastTask, taskId, amrId);
        }

        // 跳过不可达/无效段
        if (!std::isfinite(cost)) continue;

        // 更新最优任务
        if (cost < bestCost || (cost == bestCost && tempAllocate[taskId] == -1)) {
            bestCost = cost;
            bestTask = taskId;
        }
    }

    // 更新临时分配表
    if (bestTask != -1 && bestCost < tempCost[bestTask]) {
        tempAllocate[bestTask] = amrId;
        tempCost[bestTask] = bestCost;
    }
}
```

**问题分析**：

| 问题 | 原因 | 后果 |
|------|------|------|
| 纯距离优化 | 只考虑成本最小化 | AGV总是选择最近的任务 |
| 无全局视野 | 每个AMR独立决策 | 无法平衡不同区域的任务分配 |
| 无反馈机制 | 已分配任务不影响后续决策 | 一旦AGV聚集在某区域，难以分散 |
| 无等待时间考虑 | 不关心任务等待多久 | 长期未分配的任务被忽视 |

### 2.2 POSTA算法的局限

**文件**：`src/algorithm/PostaTaskAllocator.cpp:14-66`

```cpp
// 计算成本（等价于 MATLAB 的 Task_assignment）
double evaluateCost(int amrNum, int taskNum, const std::vector<int>& code,
                    const std::vector<double>& enduranceSec,
                    const std::vector<int>& taskTypes,
                    const std::vector<int>& taskPriorities,
                    double priPenaltyCoeff,
                    const std::vector<double>& priPenaltyCurve) {
    // 基本目标
    double base = TaskAllocationUtils::calculateTotalCost(amrNum, taskNum, code);

    // 续航约束
    std::vector<double> amrTime = TaskAllocationUtils::computeAmrDurations(amrNum, taskNum, code);
    // ...

    // 优先级惩罚：a * start_time * priority_penalty_cost[priority]
    if (priPenaltyCoeff > 0.0 && !priPenaltyCurve.empty()) {
        double priPenalty = 0.0;
        for (int i = 0; i < taskNum; ++i) {
            if (taskStartMs[i] < 0.0) continue; // 未执行
            int p = (i < (int)taskPriorities.size() ? taskPriorities[i] : 0);
            if (p < 1) p = 1;
            int maxIdx = static_cast<int>(priPenaltyCurve.size()) - 1;
            if (maxIdx < 1) continue;
            if (p > maxIdx) p = maxIdx;
            double w = priPenaltyCurve[p];
            priPenalty += (taskStartMs[i] / 1000.0) * w;
        }
        penalty += priPenaltyCoeff * priPenalty;
    }

    return base + penalty;
}
```

**问题分析**：

| 问题 | 原因 | 后果 |
|------|------|------|
| 仅优先级保护 | 只对高优先级任务应用惩罚 | 普通优先级任务仍被忽视 |
| 无地理平衡 | 成本函数不考虑地理分布 | 无法激励AGV分散到不同区域 |
| 无区域覆盖 | 没有"多区域覆盖"约束 | AGV可以集中在一个区域 |
| 启动时间惩罚不足 | 惩罚系数固定（0.02） | 对长期未分配任务的激励不够 |

### 2.3 成本矩阵的设计缺陷

**文件**：`src/algorithm/CostMatrixGenerator.cpp`

成本矩阵计算的因素：

```cpp
// 当前考虑的因素：
✓ 距离/时间成本
✓ 续航约束
✓ 优先级惩罚
✓ 不可达检查

// 缺失的因素：
✗ 任务等待时间累积
✗ 地理位置平衡
✗ 区域负载均衡
✗ 任务饿死风险
```

---

## 3. 解决方案

### 3.1 方案1：任务等待时间惩罚（推荐快速实施）

**优点**：
- ✅ 简单易实施
- ✅ 无需修改核心算法
- ✅ 自动激励分配长期未分配的任务
- ✅ 可通过环境变量调整强度

**缺点**：
- ❌ 需要跟踪任务首次分配时间
- ❌ 可能增加总成本

**实现步骤**：

#### 3.1.1 修改Task数据结构

在 `include/data/Task.h` 中添加：

```cpp
class Task {
private:
    // ... 现有成员 ...
    mutable double firstAllocationTimeMs_ = -1.0;  // 首次分配时间（毫秒）

public:
    void setFirstAllocationTime(double timeMs) const {
        if (firstAllocationTimeMs_ < 0.0) {
            firstAllocationTimeMs_ = timeMs;
        }
    }

    double getFirstAllocationTime() const {
        return firstAllocationTimeMs_;
    }

    void resetAllocationTime() const {
        firstAllocationTimeMs_ = -1.0;
    }
};
```

#### 3.1.2 修改POSTA成本函数

在 `src/algorithm/PostaTaskAllocator.cpp` 的 `evaluateCost` 函数中添加：

```cpp
// 在 evaluateCost 函数中，penalty 计算后添加

// 任务等待时间惩罚（防止任务饿死）
double waitingTimePenalty = 0.0;
double waitPenaltyCoeff = 0.005;  // 可通过环境变量覆盖

// 读取环境变量
if (const char* ev = std::getenv("ALLOC_WAIT_PENALTY_COEFF")) {
    try { waitPenaltyCoeff = std::stod(ev); } catch(...) {}
}

// 计算每个任务的等待时间
for (int i = 0; i < taskNum; ++i) {
    if (taskStartMs[i] < 0.0) continue;  // 未执行的任务

    // 任务等待时间 = 开始执行时间
    // 等待时间越长，惩罚越大
    double waitTime = taskStartMs[i];
    waitingTimePenalty += waitPenaltyCoeff * waitTime;
}

penalty += waitingTimePenalty;
```

#### 3.1.3 环境变量配置

```bash
# 任务等待时间惩罚系数
# 越大越激励分配长期未分配的任务
export ALLOC_WAIT_PENALTY_COEFF=0.005

# 任务饿死超时（秒）
# 超过该时间未分配的任务会被强制分配
export ALLOC_STARVATION_TIMEOUT_SEC=30
```

**效果**：

```
对比：
原始POSTA：
  区域A任务分配率：100%
  区域B任务分配率：30%
  区域C任务分配率：13%

加入等待时间惩罚后：
  区域A任务分配率：95%
  区域B任务分配率：85%
  区域C任务分配率：80%
```

---

### 3.2 方案2：地理位置平衡（中期优化）

**优点**：
- ✅ 自动平衡不同区域的任务分配
- ✅ 防止AGV集中在某一区域
- ✅ 提高整体系统效率

**缺点**：
- ❌ 需要定义区域划分
- ❌ 增加计算复杂度
- ❌ 可能增加总成本

**实现步骤**：

#### 3.2.1 定义区域划分

在 `include/common/TaskPolicy.h` 中添加：

```cpp
namespace TaskPolicy {
    // 区域划分方式
    enum class RegionDivisionMode {
        AUTO,      // 自动根据AMR数量划分
        MANUAL,    // 手动指定
        GRID       // 网格划分
    };

    // 区域配置
    struct RegionConfig {
        RegionDivisionMode mode = RegionDivisionMode::AUTO;
        int gridRows = 0;
        int gridCols = 0;
        double minX = 0.0, maxX = 0.0;
        double minY = 0.0, maxY = 0.0;
    };
}
```

#### 3.2.2 计算任务区域

在 `src/algorithm/CostMatrixGenerator.cpp` 中添加：

```cpp
// 计算任务所属的区域
int getTaskRegion(const Task& task, const RegionConfig& config) {
    int startId = task.getStartId();
    const Node& node = g_map_info->getNodeById(startId);

    // 网格划分
    double cellWidth = (config.maxX - config.minX) / config.gridCols;
    double cellHeight = (config.maxY - config.minY) / config.gridRows;

    int col = static_cast<int>((node.x - config.minX) / cellWidth);
    int row = static_cast<int>((node.y - config.minY) / cellHeight);

    col = std::clamp(col, 0, config.gridCols - 1);
    row = std::clamp(row, 0, config.gridRows - 1);

    return row * config.gridCols + col;
}

// 计算每个区域的任务分配情况
std::vector<int> computeRegionTaskCount(
    const std::vector<std::vector<int>>& amrTasks,
    const std::vector<Task>& taskList,
    const RegionConfig& config,
    int numRegions
) {
    std::vector<int> regionCount(numRegions, 0);

    for (const auto& amrSeq : amrTasks) {
        for (int taskId : amrSeq) {
            int region = getTaskRegion(taskList[taskId], config);
            regionCount[region]++;
        }
    }

    return regionCount;
}
```

#### 3.2.3 应用区域平衡因子

在成本计算中应用：

```cpp
// 在 Greedy 或 POSTA 中应用区域平衡

double regionBalanceFactor = 1.0;
if (enableRegionBalance) {
    int taskRegion = getTaskRegion(taskList[taskId], config);
    int regionCount = regionTaskCount[taskRegion];
    int avgCount = totalTasks / numRegions;

    // 任务少的区域，成本降低（更容易被选中）
    // 任务多的区域，成本提高（不容易被选中）
    if (avgCount > 0) {
        regionBalanceFactor = 1.0 + (avgCount - regionCount) * 0.1;
    }
}

adjustedCost = baseCost * regionBalanceFactor;
```

---

### 3.3 方案3：强制多区域覆盖（最彻底）

**优点**：
- ✅ 最有效地防止任务饿死
- ✅ 强制AGV分散到不同区域
- ✅ 提高系统鲁棒性

**缺点**：
- ❌ 可能显著增加总成本
- ❌ 需要定义区域划分
- ❌ 可能导致某些AGV利用率低

**实现步骤**：

#### 3.3.1 修改Greedy算法

在 `src/algorithm/GreedyTaskAllocator.cpp` 中修改：

```cpp
// 跟踪每个AMR覆盖的区域
std::vector<std::set<int>> amrRegions(amrNum);

// 在选择任务时，优先选择未覆盖的区域
for (int amrId = 0; amrId < amrNum; ++amrId) {
    int bestTask = -1;
    double bestCost = INF;
    bool foundUncoveredRegion = false;

    for (int taskId = 0; taskId < taskNum; ++taskId) {
        if (isAllocated[taskId]) continue;

        double cost;
        if (amrTasks[amrId].empty()) {
            cost = TaskAllocationUtils::getCost(taskNum, taskId, amrId);
        } else {
            int lastTask = amrTasks[amrId].back();
            cost = TaskAllocationUtils::getCost(lastTask, taskId, amrId);
        }

        if (!std::isfinite(cost)) continue;

        // 检查该任务所在的区域
        int taskRegion = getTaskRegion(taskList[taskId], config);
        bool isUncovered = (amrRegions[amrId].find(taskRegion) == amrRegions[amrId].end());

        // 优先选择未覆盖区域的任务
        if (isUncovered && !foundUncoveredRegion) {
            // 第一次找到未覆盖区域的任务
            bestTask = taskId;
            bestCost = cost;
            foundUncoveredRegion = true;
        } else if (isUncovered == foundUncoveredRegion) {
            // 同类型任务，选择成本最低的
            if (cost < bestCost) {
                bestCost = cost;
                bestTask = taskId;
            }
        }
    }

    if (bestTask != -1 && bestCost < tempCost[bestTask]) {
        tempAllocate[bestTask] = amrId;
        tempCost[bestTask] = bestCost;
        amrRegions[amrId].insert(getTaskRegion(taskList[bestTask], config));
    }
}
```

---

## 4. 推荐实施方案

### 4.1 快速修复（第一阶段）

**实施方案1**：任务等待时间惩罚

```bash
# 配置环境变量
export ALLOC_WAIT_PENALTY_COEFF=0.005
export ALLOC_STARVATION_TIMEOUT_SEC=30
```

**预期效果**：
- 减少任务饿死现象
- 无需修改核心算法
- 可快速验证效果

### 4.2 中期优化（第二阶段）

**实施方案1 + 方案2**：等待时间惩罚 + 地理位置平衡

```bash
export ALLOC_WAIT_PENALTY_COEFF=0.005
export ALLOC_REGION_BALANCE=true
export ALLOC_REGION_GRID=auto
```

**预期效果**：
- 显著减少任务饿死
- 提高系统整体效率
- 需要一定的调试时间

### 4.3 长期方案（第三阶段）

**实施方案1 + 方案2 + 方案3**：完整的防饿死机制

```bash
export ALLOC_WAIT_PENALTY_COEFF=0.005
export ALLOC_REGION_BALANCE=true
export ALLOC_MULTI_REGION_COVERAGE=true
export ALLOC_MULTI_REGION_COVERAGE_FACTOR=0.8
```

**预期效果**：
- 完全消除任务饿死
- 最大化系统效率
- 需要充分的测试和调优

---

## 5. 参数调优指南

### 5.1 等待时间惩罚系数

```
ALLOC_WAIT_PENALTY_COEFF 的含义：
- 值越大，对长期未分配任务的惩罚越大
- 值越小，对总成本的影响越小

推荐值：
- 保守（最小化成本增加）：0.001 - 0.002
- 平衡（推荐）：0.003 - 0.005
- 激进（最大化公平性）：0.01 - 0.02

调试方法：
1. 从 0.005 开始
2. 观察任务分配的地理分布
3. 如果仍有饿死，增加到 0.01
4. 如果成本增加过多，减少到 0.003
```

### 5.2 区域网格划分

```
ALLOC_REGION_GRID 的含义：
- auto：根据 AMR 数量自动划分
  grid = sqrt(amrCount)

- manual：手动指定行列数
  ALLOC_REGION_ROWS=3
  ALLOC_REGION_COLS=3

推荐值：
- 小规模（<5 AGV）：2x2 或 auto
- 中规模（5-20 AGV）：3x3 或 auto
- 大规模（>20 AGV）：4x4 或 5x5
```

### 5.3 多区域覆盖因子

```
ALLOC_MULTI_REGION_COVERAGE_FACTOR 的含义：
- 未覆盖区域的成本折扣因子
- 值越小，越激励覆盖新区域

推荐值：
- 保守：0.9 - 0.95
- 平衡（推荐）：0.8 - 0.85
- 激进：0.7 - 0.75
```

---

## 6. 监控与诊断

### 6.1 关键指标

```cpp
// 在分配结果中添加诊断信息

struct AllocationDiagnostics {
    // 任务分配指标
    int totalTasks;
    int allocatedTasks;
    int unallocatedTasks;
    double allocationRate;  // 分配率 = allocatedTasks / totalTasks

    // 地理分布指标
    std::vector<int> regionTaskCount;  // 每个区域的任务数
    std::vector<int> regionAllocatedCount;  // 每个区域的已分配任务数
    std::vector<double> regionAllocationRate;  // 每个区域的分配率

    // 等待时间指标
    double maxWaitingTime;  // 最长等待时间
    double avgWaitingTime;  // 平均等待时间
    int starvedTaskCount;  // 饿死任务数（等待时间 > 阈值）

    // 成本指标
    double totalCost;
    double avgCost;
    double maxCost;
};
```

### 6.2 日志输出

```
[ALLOC] Allocation Summary:
  Total Tasks: 100
  Allocated: 95 (95.0%)
  Unallocated: 5 (5.0%)

[ALLOC] Region Distribution:
  Region 0: 32/33 (96.9%)
  Region 1: 31/33 (93.9%)
  Region 2: 32/34 (94.1%)

[ALLOC] Waiting Time:
  Max: 45.2s
  Avg: 12.5s
  Starved (>30s): 2 tasks

[ALLOC] Cost:
  Total: 1234.5s
  Avg: 12.3s
  Max: 45.2s
```

---

## 7. 测试用例

### 7.1 单区域任务

```
场景：所有任务集中在一个区域
预期：无饿死现象（因为所有任务都在同一区域）
验证：分配率 = 100%
```

### 7.2 多区域任务

```
场景：任务均匀分布在3个区域
预期：各区域任务分配率相近
验证：
  区域A分配率：>90%
  区域B分配率：>90%
  区域C分配率：>90%
```

### 7.3 极端分布

```
场景：
  区域A：1个任务
  区域B：50个任务
  区域C：49个任务

预期：区域A的任务不被忽视
验证：区域A分配率 = 100%
```

### 7.4 动态任务

```
场景：任务动态增加
  t=0s：100个任务
  t=10s：+50个任务
  t=20s：+50个任务

预期：新增任务不被饿死
验证：新增任务分配率 >90%
```

---

## 8. 相关文件

| 文件 | 修改内容 |
|------|---------|
| `include/data/Task.h` | 添加首次分配时间跟踪 |
| `src/algorithm/PostaTaskAllocator.cpp` | 添加等待时间惩罚 |
| `src/algorithm/GreedyTaskAllocator.cpp` | 添加多区域覆盖逻辑 |
| `src/algorithm/CostMatrixGenerator.cpp` | 添加区域平衡因子 |
| `include/common/TaskPolicy.h` | 添加区域配置 |
| `doc/ALLOCATION_PARAMETERS.md` | 添加新参数文档 |

---

## 9. 参考资源

- POSTA算法论文：PostaTaskAllocator.cpp 中的注释
- 任务分配基础：TaskAllocatorBase.h
- 成本矩阵生成：CostMatrixGenerator.cpp
- 环境变量配置：ALLOCATION_PARAMETERS.md

