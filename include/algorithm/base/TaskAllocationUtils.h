#ifndef TASK_ALLOCATION_UTILS_H
#define TASK_ALLOCATION_UTILS_H

#include <vector>
#include <string>
#include <limits>
#include "data/Amr.h"
#include "data/Task.h"
#include "data/MapInfo.h"

// 任务分配通用工具类（所有算法共享的parse/deParse等函数）
class TaskAllocationUtils {
public:
    static constexpr int kCodeSeparator = -1;

    // 解析编码向量为AMR任务列表（0开始索引）
    // 输入：code（编码向量，用 kCodeSeparator 分隔不同AMR的任务，任务ID为0开始）
    // 输出：每个AMR的任务列表（保持0开始索引）
    static std::vector<std::vector<int>> parseSolution(const std::vector<int>& code);

    // 将AMR任务列表编码为向量（与parseSolution互逆）
    // 输入：amrTasks（每个AMR的任务列表，0开始索引）
    // 输出：编码向量（任务ID保持0开始，用 kCodeSeparator 分隔）
    static std::vector<int> deParseSolution(const std::vector<std::vector<int>>& amrTasks);

    // 计算任务分配的总成本（与MATLAB Task_assignment一致）
    // 参数：
    //   code: 编码向量
    //   typeMatchMatrix: AMR类型-任务类型匹配矩阵[amrTypeNum][taskTypeNum]
    //   amrTypes: 各AMR的类型（0开始）
    //   taskTypes: 各任务的类型（0开始）
    //   costMatrix: 成本矩阵[任务数+1][任务数][AMR数]（最后一行是AMR到任务的初始成本）
    static double calculateTotalCost(
        int amrNum,
        int taskNum,
        const std::vector<int>& code
    );

    // 计算每个AMR的累计时间（秒），用于续航判定（不做单位转换）
    static std::vector<double> computeAmrDurations(
        int amrNum,
        int taskNum,
        const std::vector<int>& code
    );

    // 稀疏 Cost 适配与统一读接口（第一阶段：读优先，写由生成器调用）
    // 启用/重置稀疏缓存（由 CostMatrixGenerator 调用）。若环境变量 ALLOC_COST_SPARSE=1/true，则启用稀疏优先读。
    static void sparseReset(int taskNum, int amrNum);
    // 写入一条边的代价（prevOrTaskNum == taskNum 表示 agent->task 首段）
    static void sparsePut(int prevOrTaskNum, int curIdx, int amrIdx, double value);
    // 统一读取接口：若启用稀疏且命中，则返回稀疏缓存值；否则回退到 dense 矩阵索引（若越界则返回 +INF）。
    static double getCost(int prevOrTaskNum, int curIdx, int amrIdx);

    // 时间戳工具：本地时间与消息时间校验
    // 返回当前系统时间（Unix秒，含小数）
    static double nowUnixSeconds();

    // 解析 ISO8601（UTC）到 Unix秒。支持 YYYY-MM-DDTHH:MM:SS[.sss][Z|±HH:MM|±HHMM|±HH]
    static bool parseIso8601ToUnixSeconds(const std::string& iso, double& outSec);

    // 从环境变量读取最大允许偏差（秒），默认0.5。
    // 优先级：TS_MAX_SKEW_SEC > ALLOC_TS_MAX_SKEW_SEC > EXT_TS_MAX_SKEW_SEC
    static double getMaxTimestampSkewSec(double defaultSec = 0.5);

    // 校验消息时间戳是否可信（|now-msg|<=maxSkewSec）。可选输出绝对秒差。
    static bool isTimestampTrusted(const std::string& isoTs,
                                   double maxSkewSec,
                                   double* outAbsDeltaSec = nullptr);

    // 供成本矩阵/修复阶段共享的全局速度（mm/s），用于把旅行时间换算为距离。
    static void setGlobalSpeedMmPerSec(double v);
    static double getGlobalSpeedMmPerSec();

    // 供 detour 截断阶段使用的地图访问（用于计算两任务起点的米氏距离）
    static void setMapInfoPtr(const MapInfo* m);
    static const MapInfo* getMapInfoPtr();
    // 计算两任务起点之间的米氏距离（默认L2，单位mm），若地图不可用或node缺失返回-1
    static double minkowskiDistanceBetweenTasks(const Task& a, const Task& b, double p = 2.0);

};

#endif // TASK_ALLOCATION_UTILS_H
