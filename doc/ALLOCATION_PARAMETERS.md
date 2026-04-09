# 任务分配参数速查

该文档汇总当前工程中与“任务分配 + 路径规划”相关的关键参数，列出**名称 / 含义 / 默认值 / 配置方式**，方便联调或排查问题。

---

## 1. 路径与输入资源

| 参数/环境变量 | 默认值 | 说明 |
|---------------|--------|------|
| `MAP_FILE` | `config/south_20260107.json` | 所有可执行文件共用的地图路径。若未显式设置，`PathUtils` 会依次在当前目录、父/祖先目录（`..`、`../..`、`../../..`）以及（若定义）`$AGV_SCHED_ROOT/config` 等常见根目录下搜索该文件名。 |
| `PLANNER_STATIC_TABLE` | `config/south_20260107_all.bin` | 静态路径表路径。与 `MAP_FILE` 相同的搜索策略，未找到时会在 `config/`、`../config/`、`. ` 等目录递归查找同名文件。 |
| `AGV_SCHED_ROOT` | （未设置） | 可选：若设置，则把 `<root>/config` 自动加入上述搜索根列表，便于部署到固定前缀目录。 |

---

## 2. AMR 过滤与任务策略（TaskPolicy）

| 参数 | 默认值 / 来源 | 含义 |
|------|---------------|------|
| `TaskPolicy::EMERGENCY_LEVEL` | `10` (`include/common/TaskPolicy.h`) | 当任务优先级 ≥10 视为紧急，可让“工作中/移动中”的 AMR 回到候选列表。 |
| `TaskPolicy::MAX_TASK_NUM_TH` | `10` | 目前仅保留常量，逻辑已改为“有任务且存在空闲车”或“有紧急任务”才触发调度。 |
| `Amr.disThresholdMm` | `15000 mm`（类成员默认） | 移动中的 AMR 与紧急任务起点的曼哈顿距离超过该阈值时，允许被抢占。 |
| `ALLOC_MAX_TIME_AHEAD_SEC` | `600`（秒，内部×1000 → 默认600000ms） | 单个 AMR 任务链向前规划的最大时间窗；>0 时在修复阶段触发 detour 截断。 |
| `ALLOC_MAX_DETOUR_MM` | `0`（不截断） | 单个 AMR 任务链累计绕行距离阈值；>0 时在修复阶段触发 detour 截断。 |
| `EXT_MAX_TIME_AHEAD_SEC` | `-1`（沿用 600 秒，内部×1000） | `external_receiver` 用于覆盖 `Amr.maxTimeAheadMs`；<0 表示保持 600000ms 默认值。 |
| `EXT_MAX_DETOUR_MM` | `-1`（保持 0） | `external_receiver` detour 距离覆盖参数；<0 表示不修改。 |

> `Amr.endurance` 直接取自 `agvStatusList.endurance`（按小时），不再强制置 `0`。

## 3. 成本矩阵与最短路配置

| 参数/环境变量 | 默认值 | 说明 |
|---------------|--------|------|
| `ALLOC_WORKERS` | `4`（`external_receiver`） | `ShortestPathUpdater` 工作线程数；外部紧急预测阶段若未设该变量，会用 2 线程做临时估算。 |
| `ALLOC_TOPK_RATIO` | `0.5` | 每个 AMR 初始 Top-K 任务数量 = `round(taskNum * ratio)`，最小为 1。 |
| `ALLOC_TOPK` | `-1`（自动） | 若 >0，同步覆盖 `ALLOC_TOPK_AGENT/ALLOC_TOPK_NEXT`。 |
| `ALLOC_TOPK_AGENT` | `defaultTopK` | 覆盖单个 AMR 首段 Top-K 数量。 |
| `ALLOC_TOPK_NEXT` | `defaultTopK` | 覆盖任务间转移 Top-K 数量。 |
| `ALLOC_TOPK_COVERAGE` | `1` | 为 0 时关闭“确保每个任务至少被一个 AMR 覆盖”的补偿逻辑。 |
| `ALLOC_COST_SPARSE` | `true` | 当前仅稀疏成本缓存，`ALLOC_COST_SPARSE` 仅保留兼容标记（不会切换 dense）。 |
| `SPU directionPenaltyMm` | `3000.0` | `ShortestPathUpdater` 构造时的转向惩罚，现已与路径规划模块保持一致，所有链路统一采用 3000 mm（常量定义于 `PathPlanningConstants`，如有需要可在源码中统一调整）。 |
| `globalSpeed` | `mapInfo.getGlobalMaxSpeed()`，兜底 `1000 mm/s` | 成本矩阵在无 per-edge 限速信息时使用的速度，亦用于 detour 时间估算。 |
| `时间单位` | `毫秒` | 所有成本、路径时间、`subTasks[i].estimatedDuration`、`startTimeOffset` 以及 detour/续航比较均统一为毫秒；外部输入若仍以秒为单位需乘以 1000。 |

---

## 4. 分配算法（Greedy / POSTA）

| 参数/环境变量 | 默认值 | 说明 |
|---------------|--------|------|
| `ALLOC_TIMEOUT_MS` | `1500` | `external_receiver` 的请求默认超时，用于决定 POSTA 可用时间。如果消息体包含 `timeoutMs` 字段，则以消息值为准。 |
| `ALLOC_SEED` | `1337` | 控制 Greedy 回退和 POSTA 初始化的随机数种子，便于复现。 |
| `ALLOC_POSTA_SE` | `30` | POSTA 每轮生成的 candidate 解数量。 |
| `ALLOC_POSTA_SWEEPS` | `10` | 每一类算子（swap/shift/sym）执行的 sweep 次数。 |
| `ALLOC_PRI_COEFF` | `0.02` | POSTA 优先级惩罚系数，结合 `priorityPenaltyCurve = {0.4,0.5,0.7,0.9,1.2,1.6,2.1,2.7,3.4,4.2}`。 |
| `posta.setTimeLimit()` | `remainSec` | 集成/外部链路都以 `max(0.01, timeoutSec - greedyElapsed - 安全裕度)` 作为 POSTA 允许时间。 |
| `posta.setBanShuffle()` | `false` | 若需要锁定任务段，仅在调试场景手动打开禁止 shuffle。 |

Greedy 始终作为初始解（不依赖环境变量），随后 POSTA 在修复阶段继续使用 `TaskAllocationUtils::repairInvalidDuties` 的 detour / 不可达 / 补洞逻辑。

---

## 5. 结果发布与路径规划

| 参数/环境变量 | 默认值 | 说明 |
|---------------|--------|------|
| `ASSIGN_RESULT_EXCHANGE` | `AlgoToDispExchange` | 分配 + 路径规划结果交换机（fanout/持久化）。 |
| `ASSIGN_RESULT_QUEUE` | `AlgoToDispQueue` | 结果队列（持久化，与 MQ 现有属性保持一致）。 |
| `ASSIGN_RESULT_ROUTING_KEY` | `AssignmentTaskResponse` | 路由键（用于标识结果类型）。 |
| `ASSIGN_RESULT_BINDING_KEY` | `#` | 队列与交换机绑定时使用的键。 |
| `ASSIGN_RESULT_TTL_MS` | 未设置 | 结果消息 TTL；>0 时才向 RabbitMQ 声明 `x-message-ttl`。 |
| `ResultPublisher` 路径规划 | 自动 | 发布结果前会调用 `GlobalPathPlanner`，静态表路径继承 `PLANNER_STATIC_TABLE`。静态表缺失时自动回退混合规划（静态表 + A*）。 |

---

## 6. Detour/时间参数打印建议

运行 `external_receiver` 时，日志中会输出调度配置，如：

```
[POSTA Params] SE=30 sweeps=10 pri_coeff=0.02 | workers=4 timeoutSec(remain)=1.45 | topk_ratio=0.10 topk_agent=(auto) topk_next=(auto)
[Detour Params] maxAheadMs=600000 maxDetourMm=0 (env: ALLOC_MAX_TIME_AHEAD_SEC / ALLOC_MAX_DETOUR_MM)
```

如需在自定义工具中打印 detour 配置，可直接读取 `Amr.getMaxTimeAheadMs()` / `Amr.getMaxDetourDistanceMm()`，或复用 `TaskAllocationUtils::getGlobalSpeedMmPerSec()` 计算触发阈值。

---

## 7. 通信时间戳校验

| 参数/环境变量 | 默认值 | 说明 |
|---------------|--------|------|
| `TS_MAX_SKEW_SEC` | `0.5` 秒 | 全局最大允许时间戳偏差。requestTimestamp 与本地时间差超过该值则直接丢弃消息。 |
| `ALLOC_TS_MAX_SKEW_SEC` | 未设置 | 旧名覆盖项；优先级低于 `TS_MAX_SKEW_SEC`，高于 `EXT_TS_MAX_SKEW_SEC`，external_receiver 仍会读取。 |
| `EXT_TS_MAX_SKEW_SEC` | 未设置 | `external_receiver` 专用覆盖项；仅在前两者未设置时生效。 |

校验逻辑位于 `TaskAllocationUtils::isTimestampTrusted()`，默认限制 ±0.5 秒，可通过上述环境变量放宽或收紧。配置后，日志会打印生效阈值并提示原因（见 `external_receiver.cpp:720+`）。

---

## 8. 任务饿死防护参数（Task Starvation Prevention）

### 8.1 问题背景

在多AGV调度中，由于贪心算法的局部最优特性，可能导致某些区域的任务长期无人接取（任务饿死）。具体表现为：
- AGV聚集在某一区域，持续接取该区域的近任务
- 其他区域的任务因距离远而被忽视
- 最终导致系统效率下降

详见 `doc/TASK_STARVATION_PREVENTION.md`。

### 8.2 防护参数

| 参数/环境变量 | 默认值 | 说明 |
|---------------|--------|------|
| `ALLOC_WAIT_PENALTY_COEFF` | `0`（未设置则关闭；`script/start.sh` 默认设为 `0.5`） | 任务等待时间惩罚系数。越大越激励分配长期未分配的任务。推荐范围：0.05-1.0。详见 8.3 节。 |
| `ALLOC_STARVATION_TIMEOUT_SEC` | `30` | 任务未分配超过该时间则被视为"饿死风险"。用于监控和诊断，不直接影响分配算法。 |
| `ALLOC_REGION_BALANCE` | `false` | 是否启用地理位置平衡。启用后，会根据区域任务分配情况调整成本。 |
| `ALLOC_REGION_GRID` | `auto` | 区域划分方式。`auto` 时根据 AMR 数量自动划分（grid = sqrt(amrCount)）；`manual` 时需指定 `ALLOC_REGION_ROWS` 和 `ALLOC_REGION_COLS`。 |
| `ALLOC_REGION_ROWS` | `0`（自动） | 手动指定区域网格的行数（仅当 `ALLOC_REGION_GRID=manual` 时生效）。 |
| `ALLOC_REGION_COLS` | `0`（自动） | 手动指定区域网格的列数（仅当 `ALLOC_REGION_GRID=manual` 时生效）。 |
| `ALLOC_MULTI_REGION_COVERAGE` | `false` | 是否启用强制多区域覆盖。启用后，Greedy 会优先为每个 AMR 分配不同区域的任务。 |
| `ALLOC_MULTI_REGION_COVERAGE_FACTOR` | `0.8` | 多区域覆盖的成本折扣因子。未覆盖区域的任务成本乘以该因子（越小越激励覆盖新区域）。 |

### 8.3 参数调优指南

#### 8.3.1 等待时间惩罚系数（ALLOC_WAIT_PENALTY_COEFF）

**含义（量纲）**：
- 该项直接加到分配“成本”里；POSTA 的基础成本是**秒**（avg + max 时长）
- 等待惩罚 = `ALLOC_WAIT_PENALTY_COEFF * waitSec`（单位仍是秒）
- 因此系数可理解为：**等待 1 秒增加多少“成本秒”**

**推荐值（按量纲调参）**：
```
温和：0.05 - 0.2
  适用：成本敏感，仍希望缓解饿死
  例：wait 10s → 0.5~2s 成本

均衡：0.2 - 0.6
  适用：多数仿真/生产调参
  例：wait 10s → 2~6s 成本

激进：0.6 - 1.0
  适用：强公平性，允许总成本上升
  例：wait 10s → 6~10s 成本
```

**调试方法**：
```bash
# 1. 从默认值开始（start.sh 默认 0.5）
export ALLOC_WAIT_PENALTY_COEFF=0.5

# 2. 运行仿真，观察任务分配的地理分布
# 查看日志中的 [ALLOC] Region Distribution 部分

# 3. 如果仍有饿死现象，增加系数
export ALLOC_WAIT_PENALTY_COEFF=0.8

# 4. 如果成本增加过多，减少系数
export ALLOC_WAIT_PENALTY_COEFF=0.2
```

#### 8.3.2 区域网格划分（ALLOC_REGION_GRID）

**自动划分**：
```bash
export ALLOC_REGION_GRID=auto

# 自动计算：grid = sqrt(amrCount)
# 示例：
#   2 AGV  → 1x1 (无效，改为 2x1)
#   4 AGV  → 2x2
#   9 AGV  → 3x3
#   16 AGV → 4x4
```

**手动划分**：
```bash
export ALLOC_REGION_GRID=manual
export ALLOC_REGION_ROWS=3
export ALLOC_REGION_COLS=3

# 推荐值：
# 小规模（<5 AGV）：2x2 或 auto
# 中规模（5-20 AGV）：3x3 或 auto
# 大规模（>20 AGV）：4x4 或 5x5
```

#### 8.3.3 多区域覆盖因子（ALLOC_MULTI_REGION_COVERAGE_FACTOR）

**含义**：
- 未覆盖区域的任务成本折扣
- 值越小，越激励 AMR 覆盖新区域
- 范围：0.5 - 1.0

**推荐值**：
```
保守：0.9 - 0.95
  效果：轻微激励覆盖新区域

平衡（推荐）：0.8 - 0.85
  效果：明显激励覆盖新区域

激进：0.7 - 0.75
  效果：强烈激励覆盖新区域
```

**配置示例**：
```bash
export ALLOC_MULTI_REGION_COVERAGE=true
export ALLOC_MULTI_REGION_COVERAGE_FACTOR=0.8
```

### 8.4 完整配置示例

#### 快速修复（推荐第一阶段）

```bash
# 仅启用等待时间惩罚
export ALLOC_WAIT_PENALTY_COEFF=0.005
export ALLOC_STARVATION_TIMEOUT_SEC=30
```

**预期效果**：
- 减少任务饿死现象
- 成本增加 <5%
- 无需修改核心算法

#### 中期优化（推荐第二阶段）

```bash
# 等待时间惩罚 + 地理位置平衡
export ALLOC_WAIT_PENALTY_COEFF=0.005
export ALLOC_REGION_BALANCE=true
export ALLOC_REGION_GRID=auto
export ALLOC_STARVATION_TIMEOUT_SEC=30
```

**预期效果**：
- 显著减少任务饿死
- 提高系统整体效率
- 成本增加 5-10%

#### 长期方案（推荐第三阶段）

```bash
# 完整的防饿死机制
export ALLOC_WAIT_PENALTY_COEFF=0.005
export ALLOC_REGION_BALANCE=true
export ALLOC_REGION_GRID=auto
export ALLOC_MULTI_REGION_COVERAGE=true
export ALLOC_MULTI_REGION_COVERAGE_FACTOR=0.8
export ALLOC_STARVATION_TIMEOUT_SEC=30
```

**预期效果**：
- 完全消除任务饿死
- 最大化系统效率
- 成本增加 10-15%

### 8.5 监控指标

运行时日志会输出以下诊断信息：

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

**关键指标解释**：
- **分配率**：已分配任务数 / 总任务数。目标 >95%
- **区域分配率**：每个区域的分配率应相近（差异 <10%）
- **饿死任务数**：等待时间 > `ALLOC_STARVATION_TIMEOUT_SEC` 的任务数。目标 = 0
- **成本增加**：与基线相比的成本增加百分比。目标 <15%

### 8.6 故障排查

#### 问题：仍有任务饿死

**原因**：
1. 惩罚系数过小
2. 区域划分不合理
3. 任务分布极度不均

**解决方案**：
```bash
# 1. 增加惩罚系数
export ALLOC_WAIT_PENALTY_COEFF=0.01

# 2. 启用地理位置平衡
export ALLOC_REGION_BALANCE=true

# 3. 启用多区域覆盖
export ALLOC_MULTI_REGION_COVERAGE=true
```

#### 问题：成本增加过多（>20%）

**原因**：
1. 惩罚系数过大
2. 多区域覆盖因子过小
3. 区域划分导致任务分散

**解决方案**：
```bash
# 1. 减少惩罚系数
export ALLOC_WAIT_PENALTY_COEFF=0.002

# 2. 增加覆盖因子
export ALLOC_MULTI_REGION_COVERAGE_FACTOR=0.9

# 3. 调整区域划分
export ALLOC_REGION_ROWS=2
export ALLOC_REGION_COLS=2
```

#### 问题：某个区域的分配率特别低

**原因**：
1. 该区域任务距离远
2. 该区域任务优先级低
3. 区域划分不合理

**解决方案**：
```bash
# 1. 检查任务优先级
# 为该区域的任务提高优先级

# 2. 调整区域划分
# 使该区域更接近 AMR 初始位置

# 3. 增加多区域覆盖强度
export ALLOC_MULTI_REGION_COVERAGE_FACTOR=0.7
```

---

## 9. 参数优先级与覆盖关系

```
环境变量优先级（从高到低）：
1. 直接环境变量（如 ALLOC_WAIT_PENALTY_COEFF）
2. 消息体中的参数（如 timeoutMs）
3. 代码中的默认值

示例：
export ALLOC_WAIT_PENALTY_COEFF=0.01
# 覆盖代码中的默认值 0.005
```

---

## 10. 相关文档

- **详细分析**：`doc/TASK_STARVATION_PREVENTION.md`
- **算法实现**：`src/algorithm/PostaTaskAllocator.cpp`
- **成本矩阵**：`src/algorithm/CostMatrixGenerator.cpp`
- **基础工具**：`include/algorithm/base/TaskAllocationUtils.h`
