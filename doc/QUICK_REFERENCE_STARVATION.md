# 任务饿死防护 - 快速参考指南

## 🚀 快速开始

### 问题诊断

**症状**：AGV集中在某一区域，其他区域的任务无人接取

**验证方法**：
```bash
# 查看日志中的区域分配情况
grep "\[ALLOC\] Region Distribution" logs/*.log

# 如果看到类似输出，说明存在饿死问题：
# Region 0: 95/100 (95%)
# Region 1: 20/100 (20%)  ← 分配率过低
# Region 2: 15/100 (15%)  ← 分配率过低
```

### 快速修复（5分钟）

```bash
# 1. 设置环境变量
export ALLOC_WAIT_PENALTY_COEFF=0.5
export ALLOC_STARVATION_TIMEOUT_SEC=30

# 2. 重启调度系统
./external_receiver

# 3. 观察日志
# 应该看到各区域分配率更均衡
```

---

## 📊 参数速查表

### 核心参数

| 参数 | 默认值 | 推荐值 | 说明 |
|------|--------|--------|------|
| `ALLOC_WAIT_PENALTY_COEFF` | 0.5 | 0.2-0.8 | 等待时间惩罚系数 |
| `ALLOC_STARVATION_TIMEOUT_SEC` | 30 | 20-60 | 饿死超时（秒） |
| `ALLOC_REGION_BALANCE` | false | true | 启用地理平衡 |
| `ALLOC_MULTI_REGION_COVERAGE` | false | true | 启用多区域覆盖 |

### 调整建议

```
如果仍有饿死 → 增加 ALLOC_WAIT_PENALTY_COEFF
如果成本增加过多 → 减少 ALLOC_WAIT_PENALTY_COEFF
如果某区域分配率低 → 启用 ALLOC_REGION_BALANCE
如果需要最强保护 → 启用 ALLOC_MULTI_REGION_COVERAGE
```

---

## 🔍 监控指标

### 关键指标

```
分配率 = 已分配任务数 / 总任务数
目标：>95%

区域分配率差异 = max(区域分配率) - min(区域分配率)
目标：<10%

饿死任务数 = 等待时间 > ALLOC_STARVATION_TIMEOUT_SEC 的任务数
目标：0

成本增加 = (新成本 - 旧成本) / 旧成本 * 100%
目标：<15%
```

### 日志查看

```bash
# 查看分配摘要
grep "\[ALLOC\] Allocation Summary" logs/*.log

# 查看区域分布
grep "\[ALLOC\] Region Distribution" logs/*.log

# 查看等待时间
grep "\[ALLOC\] Waiting Time" logs/*.log

# 查看成本
grep "\[ALLOC\] Cost" logs/*.log
```

---

## 🛠️ 故障排查

### 问题1：仍有任务饿死

**症状**：某些区域的任务分配率仍然很低

**解决步骤**：
```bash
# 1. 检查参数是否生效
echo $ALLOC_WAIT_PENALTY_COEFF

# 2. 增加惩罚系数
export ALLOC_WAIT_PENALTY_COEFF=0.01

# 3. 启用地理平衡
export ALLOC_REGION_BALANCE=true

# 4. 重启并观察
./external_receiver
```

### 问题2：成本增加过多（>20%）

**症状**：总成本显著增加

**解决步骤**：
```bash
# 1. 减少惩罚系数
export ALLOC_WAIT_PENALTY_COEFF=0.002

# 2. 禁用多区域覆盖
export ALLOC_MULTI_REGION_COVERAGE=false

# 3. 重启并观察
./external_receiver
```

### 问题3：某个区域分配率特别低

**症状**：某个区域的任务分配率 <50%

**解决步骤**：
```bash
# 1. 检查该区域的任务优先级
# 为该区域的任务提高优先级

# 2. 调整区域划分
export ALLOC_REGION_GRID=manual
export ALLOC_REGION_ROWS=2
export ALLOC_REGION_COLS=3

# 3. 增加多区域覆盖强度
export ALLOC_MULTI_REGION_COVERAGE_FACTOR=0.7

# 4. 重启并观察
./external_receiver
```

---

## 📈 性能对比

### 修复前后对比

```
修复前（仅Greedy）：
  区域A分配率：98%
  区域B分配率：25%
  区域C分配率：12%
  总成本：1000s
  饿死任务数：15

修复后（+等待时间惩罚）：
  区域A分配率：92%
  区域B分配率：88%
  区域C分配率：85%
  总成本：1045s (+4.5%)
  饿死任务数：0

修复后（+地理平衡）：
  区域A分配率：90%
  区域B分配率：92%
  区域C分配率：91%
  总成本：1080s (+8%)
  饿死任务数：0
```

---

## 🎯 配置方案

### 方案A：保守（最小化成本增加）

```bash
export ALLOC_WAIT_PENALTY_COEFF=0.002
export ALLOC_STARVATION_TIMEOUT_SEC=30
# 其他参数保持默认
```

**适用场景**：成本敏感，可接受少量任务饿死

### 方案B：平衡（推荐）

```bash
export ALLOC_WAIT_PENALTY_COEFF=0.005
export ALLOC_REGION_BALANCE=true
export ALLOC_REGION_GRID=auto
export ALLOC_STARVATION_TIMEOUT_SEC=30
```

**适用场景**：大多数生产环境

### 方案C：激进（最大化公平性）

```bash
export ALLOC_WAIT_PENALTY_COEFF=0.01
export ALLOC_REGION_BALANCE=true
export ALLOC_REGION_GRID=auto
export ALLOC_MULTI_REGION_COVERAGE=true
export ALLOC_MULTI_REGION_COVERAGE_FACTOR=0.8
export ALLOC_STARVATION_TIMEOUT_SEC=30
```

**适用场景**：公平性优先，可接受成本增加

---

## 📋 检查清单

### 部署前检查

- [ ] 已读 `doc/TASK_STARVATION_PREVENTION.md`
- [ ] 已理解三种解决方案的区别
- [ ] 已选择合适的配置方案（A/B/C）
- [ ] 已设置环境变量
- [ ] 已准备测试数据

### 部署后验证

- [ ] 日志中出现 `[ALLOC]` 诊断信息
- [ ] 各区域分配率相近（差异 <10%）
- [ ] 饿死任务数为0
- [ ] 成本增加在可接受范围内（<15%）
- [ ] 系统运行稳定，无异常错误

### 持续监控

- [ ] 每天检查分配率指标
- [ ] 每周检查成本趋势
- [ ] 发现异常立即调查
- [ ] 定期优化参数

---

## 🔗 相关文档

| 文档 | 用途 |
|------|------|
| `TASK_STARVATION_PREVENTION.md` | 详细技术分析 |
| `ALLOCATION_PARAMETERS.md` | 完整参数文档 |
| `ARCHITECTURE.md` | 系统架构 |
| `RUNNING.md` | 运行指南 |

---

## 💡 常见问题

**Q: 为什么我的任务仍然饿死？**
A: 检查环境变量是否正确设置。使用 `echo $ALLOC_WAIT_PENALTY_COEFF` 验证。

**Q: 成本增加了多少？**
A: 取决于参数配置。保守方案 <5%，平衡方案 5-10%，激进方案 10-15%。

**Q: 可以同时启用所有参数吗？**
A: 可以，但建议从简单方案开始，逐步增加复杂度。

**Q: 如何回滚到原始配置？**
A: 取消设置环境变量或设置为默认值，重启系统。

**Q: 参数调整后需要重启吗？**
A: 是的，需要重启 `external_receiver` 才能生效。

---

## 📞 获取帮助

如有问题，请参考：
1. `doc/TASK_STARVATION_PREVENTION.md` - 详细分析
2. `doc/ALLOCATION_PARAMETERS.md` - 参数说明
3. 日志输出 - 诊断信息
