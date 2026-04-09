# 多源车辆状态仲裁（Status Arbitration）

## 解决什么问题

在真实联调里，`external_receiver` 往往会同时收到两类“车辆状态”：

1. **实时状态流**：例如 `SendRobotStatusInfos`（连续上报，频率高，通常最可靠）
2. **任务请求快照**：例如 `AssignmentTaskRequest` 里携带的 `agvStatusList`（用于“把任务和当时的车状态打包发送”，但可能是**滞后/秒级时间戳/乱序**）

如果直接用“最后一次写入覆盖”（last-write-wins），会出现两个典型故障：

- **定位/锚点回退导致“跳点/瞬移”**：旧的 `agvStatusList` 覆盖了更“新”的实时状态，导致 `RobotPath/RobotTrailResponse` 的锚点节点回到过去，消费者看到车辆跳来跳去。
- **调度误判为忙导致“无限 WAIT”**：调度阶段使用了陈旧状态（或被陈旧状态覆盖后的状态），把空闲车误判为作业中/移动中，出现 `doPlan=0` 或 `availableAmrs=0`，结果任务长期不分配；仿真/可视化侧会看到“任务点还在，但车一直不动/WAIT”。

所以需要一个“**多源状态仲裁**”组件，把多路输入归一到一份**可用于算法决策的权威状态快照**（canonical snapshot）。

---

## 组件位置与使用方式

- 头文件：`include/common/StatusArbitration.h`
- 接收端使用原则：
  - **所有状态写入都必须走仲裁**（统一调用 `ShouldAcceptStatusUpdate()` 决定是否覆盖缓存）
  - **所有算法决策只读缓存快照**（例如调度/规划构造 AMR 列表时只用 repository 的 snapshot，而不是直接读某个 payload 的 `agvStatusList`）

---

## 核心逻辑（仲裁规则）

每条状态更新都有两个关键元数据：

- `StatusSource`：来源（`STATUS_STREAM` vs `TASK_SNAPSHOT`）
- `updateTime`：统一归一化到**epoch 秒**（支持输入为秒/毫秒/ISO8601 字符串）

默认策略（`Policy`）包含两条规则：

1. **时间戳单调性（防回退）**
   - 当 `oldTs>0 && newTs>0` 且 `newTs < oldTs` 时，拒绝新状态。
   - 目的：抵抗乱序消息，避免“锚点回退/瞬移”。

2. **来源优先级（同秒/缺失时间戳时，保护实时流）**
   - 当旧状态来自 `STATUS_STREAM`，新状态来自 `TASK_SNAPSHOT` 时：
     - 只有 `newTs > oldTs` 才允许覆盖（`newTs==oldTs` 或 `newTs==0` 直接拒绝）。
   - 目的：抵抗“秒级时间戳粒度 + 批量快照滞后”，防止快照把实时状态冲掉，从而造成 `availableAmrs=0`、无限 WAIT。

这两条规则合起来保证：

- **同一辆车的状态不会“倒退”**（只要外部 updateTime 可比较）
- **实时状态流在“同秒/无时间戳”场景下不会被快照覆盖**

---

## 时间戳归一化（updateTime）

组件提供：

- `NormalizeEpochSeconds(int64 raw)`：把“秒或毫秒”统一成秒（`raw>1e11` 视为毫秒）。
- `ParseUpdateTimeSec(string_view)`：支持
  - 纯数字字符串（秒/毫秒）
  - ISO8601 UTC（如 `2026-01-08T00:40:58Z` 或带毫秒 `...58.123Z`）

归一化后再参与仲裁，可以避免“某些链路发毫秒、某些链路发秒”导致比较失真。

---

## 复用建议

任何模块只要同时接入“高频状态流 + 低频快照/批量状态”，都建议复用这套模式：

- 把状态统一写进“按 deviceId 索引的缓存”
- 通过 `StatusSource + updateTime` 做仲裁
- 下游逻辑（调度/规划/回包）只读缓存快照，不直接用某个消息里的快照字段

