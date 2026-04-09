#include "algorithm/PostaTaskAllocator.h"
#include "algorithm/base/TaskAllocationUtils.h"
#include <algorithm>
#include <chrono>
#include <limits>
#include <numeric>
#include <cmath>

namespace {

using CostMatrix3D = std::vector<std::vector<std::vector<double>>>;

std::vector<int> normalizeInitialCode(const std::vector<int>& code, int taskNum) {
    const bool hasCanonicalSeparator = std::find(
        code.begin(), code.end(), TaskAllocationUtils::kCodeSeparator) != code.end();
    if (hasCanonicalSeparator) {
        return code;
    }

    bool looksLegacyOneBased = !code.empty();
    bool hasExplicitOneBasedTaskId = false;
    for (int value : code) {
        if (value == 0) continue;
        if (value < 0 || value > taskNum) {
            looksLegacyOneBased = false;
            break;
        }
        if (value == taskNum) {
            hasExplicitOneBasedTaskId = true;
        }
    }

    if (!looksLegacyOneBased || !hasExplicitOneBasedTaskId) {
        return code;
    }

    std::vector<int> normalized;
    normalized.reserve(code.size());
    for (int value : code) {
        if (value == 0) {
            normalized.push_back(TaskAllocationUtils::kCodeSeparator);
        } else {
            normalized.push_back(value - 1);
        }
    }
    return normalized;
}

// 计算成本（等价于 MATLAB 的 Task_assignment）
double evaluateCost(int amrNum, int taskNum, const std::vector<int>& code,
                    const std::vector<double>& enduranceSec,
                    const std::vector<int>& taskTypes,
                    const std::vector<int>& taskPriorities,
                    double priPenaltyCoeff,
                    const std::vector<double>& priPenaltyCurve,
                    const std::vector<double>& taskArriveSec,
                    double baseNowSec,
                    double waitPenaltyCoeff) {
    // 基本目标
    double base = TaskAllocationUtils::calculateTotalCost(amrNum, taskNum, code);
    // 续航约束：若某AMR累计非充电任务时间超过续航，添加巨大惩罚
    std::vector<double> amrTime = TaskAllocationUtils::computeAmrDurations(amrNum, taskNum, code);
    // compute per amr excluding charging tasks时间? 我们无法区分每段是否充电任务；近似：若代码中当前任务为充电，则不计；
    // 为近似，我们重新遍历code逐AMR统计，仅累加非充电任务的段
    std::vector<double> amrNonCharge(amrNum, 0.0);
    std::vector<double> amrAccum(amrNum, 0.0); // 同步累计时间（含充电任务段），用于起始时间
    std::vector<double> taskStartMs(taskNum, -1.0);
    int currentAmr = 0; int prevTask = -1;
    for (int v : code) {
        if (v == TaskAllocationUtils::kCodeSeparator) {
            currentAmr = std::min(currentAmr + 1, amrNum - 1);
            prevTask = -1;
            continue;
        }
        int idx0 = v;
        double inc = (prevTask==-1)? TaskAllocationUtils::getCost(taskNum, idx0, currentAmr)
                                   : TaskAllocationUtils::getCost(prevTask, idx0, currentAmr);
        // 记录任务开始时间（首次出现时）
        if (idx0 >= 0 && idx0 < taskNum && taskStartMs[idx0] < 0.0) {
            taskStartMs[idx0] = amrAccum[currentAmr];
        }
        // 累计（非充电部分用于续航检查）
        if (idx0 >=0 && idx0 < (int)taskTypes.size() && taskTypes[idx0] != 3) amrNonCharge[currentAmr] += inc;
        amrAccum[currentAmr] += inc;
        prevTask = idx0;
    }
    double penalty = 0.0; const double BIG = 1e12;
    for (int i=0;i<amrNum;++i) {
        if (i < (int)enduranceSec.size() && amrNonCharge[i] > enduranceSec[i]) penalty += BIG * (amrNonCharge[i]-enduranceSec[i]+1.0);
    }

    // 优先级惩罚 & 防饿死惩罚（等待时间）
    if ((priPenaltyCoeff > 0.0 && !priPenaltyCurve.empty()) ||
        (waitPenaltyCoeff > 0.0 && baseNowSec > 0.0 && !taskArriveSec.empty())) {
        double priPenalty = 0.0;
        double waitPenalty = 0.0;
        const int maxIdx = static_cast<int>(priPenaltyCurve.size()) - 1;
        for (int i = 0; i < taskNum; ++i) {
            if (taskStartMs[i] < 0.0) continue; // 未执行
            if (priPenaltyCoeff > 0.0 && !priPenaltyCurve.empty() && maxIdx >= 1) {
                int p = (i < (int)taskPriorities.size() ? taskPriorities[i] : 0);
                if (p < 1) p = 1;
                if (p > maxIdx) p = maxIdx;
                double w = priPenaltyCurve[p];
                priPenalty += (taskStartMs[i] / 1000.0) * w;
            }
            if (waitPenaltyCoeff > 0.0 && baseNowSec > 0.0 && !taskArriveSec.empty()) {
                if (i < (int)taskArriveSec.size() && taskArriveSec[i] > 0.0) {
                    double startSec = baseNowSec + (taskStartMs[i] / 1000.0);
                    double waitSec = startSec - taskArriveSec[i];
                    if (waitSec > 0.0) waitPenalty += waitSec;
                }
            }
        }
        if (priPenaltyCoeff > 0.0 && !priPenaltyCurve.empty()) {
            penalty += priPenaltyCoeff * priPenalty;
        }
        if (waitPenaltyCoeff > 0.0) {
            penalty += waitPenaltyCoeff * waitPenalty;
        }
    }

    return base + penalty;
}

// 选择最佳个体（等价于 MATLAB 的 selection/fitness）
std::pair<std::vector<int>, double> selectBest(
    const std::vector<std::vector<int>>& states,
    int amrNum, int taskNum,
    const std::vector<double>& enduranceSec,
    const std::vector<int>& taskTypes,
    const std::vector<int>& taskPriorities,
    double priPenaltyCoeff,
    const std::vector<double>& priPenaltyCurve,
    const std::vector<double>& taskArriveSec,
    double baseNowSec,
    double waitPenaltyCoeff
) {
    double best = std::numeric_limits<double>::infinity();
    size_t bestIdx = 0;
    for (size_t i = 0; i < states.size(); ++i) {
        double f = evaluateCost(amrNum, taskNum, states[i], enduranceSec, taskTypes,
                                taskPriorities, priPenaltyCoeff, priPenaltyCurve,
                                taskArriveSec, baseNowSec, waitPenaltyCoeff);
        if (f < best) { best = f; bestIdx = i; }
    }
    return {states[bestIdx], best};
}

// 初始化解（等价于 MATLAB 的 initialization_ta），任务编码 0..taskNum-1，separator 为 kCodeSeparator
std::vector<int> initializationTa(int taskNum, int amrNum, std::mt19937& rng) {
    std::vector<int> tasks(taskNum);
    std::iota(tasks.begin(), tasks.end(), 0);
    std::shuffle(tasks.begin(), tasks.end(), rng);
    int separators = std::max(0, amrNum - 1);
    std::vector<int> seq;
    seq.reserve(taskNum + separators);
    seq.insert(seq.end(), tasks.begin(), tasks.end());
    for (int i = 0; i < separators; ++i) {
        std::uniform_int_distribution<int> dist(0, static_cast<int>(seq.size()));
        int pos = dist(rng);
        seq.insert(seq.begin() + pos, TaskAllocationUtils::kCodeSeparator);
    }
    return seq;
}

// 生成 SE 个解：在长度为 alpha 的随机段内打乱（等价于 op_swap）
std::vector<std::vector<int>> opSwap(const std::vector<int>& best, int SE, int alpha, std::mt19937& rng) {
    int n = static_cast<int>(best.size());
    alpha = std::max(1, std::min(alpha, n));
    std::vector<std::vector<int>> states(SE, best);
    std::uniform_int_distribution<int> startDist(0, n - alpha);

    for (int i = 0; i < SE; ++i) {
        int start = startDist(rng);
        int end = start + alpha; // [start, end)
        auto s = states[i];
        std::shuffle(s.begin() + start, s.begin() + end, rng);
        states[i].swap(s);
    }
    return states;
}

// 生成 SE 个解：移动长度为 alpha 的连续段到不重叠的位置（等价于 op_shift）
std::vector<std::vector<int>> opShift(const std::vector<int>& best, int SE, int alpha, std::mt19937& rng) {
    int n = static_cast<int>(best.size());
    alpha = std::max(1, std::min(alpha, n));
    std::vector<std::vector<int>> states;
    states.reserve(SE);
    std::uniform_int_distribution<int> startDist(0, n - alpha);
    for (int i = 0; i < SE; ++i) {
        int start = startDist(rng);
        int insertAfterMin = 0;
        int insertAfterMax = n - alpha; // position in terms of index before which to insert

        // choose an insert_after not in [start, start+alpha-1]
        std::uniform_int_distribution<int> posDist(insertAfterMin, insertAfterMax);
        int insertAfter = posDist(rng);
        if (insertAfter >= start && insertAfter <= start + alpha - 1) {
            // pick from left or right segment
            if (start > insertAfterMin && (rng() & 1)) {
                std::uniform_int_distribution<int> leftDist(insertAfterMin, start - 1);
                insertAfter = leftDist(rng);
            } else if (start + alpha - 1 < insertAfterMax) {
                std::uniform_int_distribution<int> rightDist(start + alpha, insertAfterMax);
                insertAfter = rightDist(rng);
            } else {
                // no valid move, keep as is
                states.push_back(best);
                continue;
            }
        }

        std::vector<int> s;
        s.reserve(n);
        // construct new order indices
        // segment indices [start, start+alpha)
        // Copy elements up to insertAfter from the rest, then segment, then the remainder
        for (int i2 = 0; i2 <= insertAfter; ++i2) {
            if (i2 < start || i2 >= start + alpha) s.push_back(best[i2]);
        }
        for (int i2 = start; i2 < start + alpha; ++i2) s.push_back(best[i2]);
        for (int i2 = insertAfter + 1; i2 < n; ++i2) {
            if (i2 < start || i2 >= start + alpha) s.push_back(best[i2]);
        }
        states.push_back(std::move(s));
    }
    return states;
}

// 生成 SE 个解：翻转长度为 alpha 的连续段（等价于 op_sym）
std::vector<std::vector<int>> opSym(const std::vector<int>& best, int SE, int alpha, std::mt19937& rng) {
    int n = static_cast<int>(best.size());
    alpha = std::max(1, std::min(alpha, n));
    std::vector<std::vector<int>> states(SE, best);
    std::uniform_int_distribution<int> startDist(0, n - alpha);
    for (int i = 0; i < SE; ++i) {
        int start = startDist(rng);
        int end = start + alpha; // [start, end)
        std::vector<int> s = states[i];
        std::reverse(s.begin() + start, s.begin() + end);
        states[i].swap(s);
    }
    return states;
}

// 保持 separator 位置不变，对若干分隔段依据 oldBest 顺序重排（等价于 op_shuffle）
std::vector<std::vector<int>> opShuffle(const std::vector<int>& oldBest, const std::vector<int>& newBest, int SE, std::mt19937& rng) {
    std::vector<std::vector<int>> states(SE, newBest);
    int n = static_cast<int>(newBest.size());

    std::vector<int> separatorIdx;
    for (int i = 0; i < n; ++i) {
        if (newBest[i] == TaskAllocationUtils::kCodeSeparator) {
            separatorIdx.push_back(i);
        }
    }
    std::vector<int> segStarts; segStarts.push_back(0);
    std::vector<int> segEnds;
    for (int idx : separatorIdx) {
        segEnds.push_back(idx - 1);
        segStarts.push_back(idx + 1);
    }
    segEnds.push_back(n - 1);

    // collect non-empty segments
    std::vector<int> segIndices;
    for (size_t i = 0; i < segStarts.size(); ++i) {
        if (segStarts[i] <= segEnds[i]) segIndices.push_back(static_cast<int>(i));
    }
    if (segIndices.empty()) return states;

    std::uniform_real_distribution<double> ratioDist(0.1, 0.6);
    for (int sIdx = 0; sIdx < SE; ++sIdx) {
        auto cur = states[sIdx];
        int toShuffle = std::max(1, static_cast<int>(std::ceil(segIndices.size() * ratioDist(rng))));
        // choose distinct segments
        std::vector<int> choice = segIndices;
        std::shuffle(choice.begin(), choice.end(), rng);
        choice.resize(toShuffle);
        for (int segId : choice) {
            int L = segStarts[segId], R = segEnds[segId];
            if (L > R) continue;
            // build pairs (oldVal, currentVal) for the segment
            std::vector<std::pair<int,int>> pairs;
            pairs.reserve(R - L + 1);
            for (int i = L; i <= R; ++i) {
                pairs.emplace_back(oldBest[i], cur[i]);
            }
            std::stable_sort(pairs.begin(), pairs.end(), [](const auto& a, const auto& b){ return a.first < b.first; });
            for (int i = L, k = 0; i <= R; ++i, ++k) cur[i] = pairs[k].second;
        }
        states[sIdx].swap(cur);
    }
    return states;
}

// 动态选择 alpha（等价于 update_alpha）
std::tuple<std::vector<int>, double, int> updateAlpha(
    const std::vector<int>& best, double fBest, const std::vector<int>& Omega, int SE,
    int amrNum, int taskNum, std::mt19937& rng,
    const std::vector<double>& enduranceSec,
    const std::vector<int>& taskTypes,
    const std::vector<int>& taskPriorities,
    double priPenaltyCoeff,
    const std::vector<double>& priPenaltyCurve,
    const std::vector<double>& taskArriveSec,
    double baseNowSec,
    double waitPenaltyCoeff
) {
    int chosenAlpha = 1;
    std::vector<int> globalBest = best;
    double globalF = fBest;
    for (int a : Omega) {
        auto states = opSwap(best, SE, a, rng);
    auto [newBest, newF] = selectBest(states, amrNum, taskNum, enduranceSec, taskTypes,
                                      taskPriorities, priPenaltyCoeff, priPenaltyCurve,
                                      taskArriveSec, baseNowSec, waitPenaltyCoeff);
        if (newF < globalF) { globalF = newF; globalBest = newBest; chosenAlpha = a; }
    }
    return {globalBest, globalF, chosenAlpha};
}

// 一次 swap 变换（等价于 swap.m）
std::pair<std::vector<int>, double> swapOnce(
    const std::vector<int>& best, double fBest, int alpha, int SE,
    int amrNum, int taskNum, std::mt19937& rng, bool banShuffle,
    const std::vector<double>& enduranceSec,
    const std::vector<int>& taskTypes,
    const std::vector<int>& taskPriorities,
    double priPenaltyCoeff,
    const std::vector<double>& priPenaltyCurve,
    const std::vector<double>& taskArriveSec,
    double baseNowSec,
    double waitPenaltyCoeff
) {
    (void)banShuffle; // not used here
    auto states = opSwap(best, SE, alpha, rng);
    auto [newBest, fNew] = selectBest(states, amrNum, taskNum, enduranceSec, taskTypes,
                                      taskPriorities, priPenaltyCoeff, priPenaltyCurve,
                                      taskArriveSec, baseNowSec, waitPenaltyCoeff);
    if (fNew < fBest) return {newBest, fNew};
    return {best, fBest};
}

// 一次 shift 变换（等价于 shift.m）
std::pair<std::vector<int>, double> shiftOnce(
    const std::vector<int>& best, double fBest, int alpha, int SE,
    int amrNum, int taskNum, std::mt19937& rng, bool banShuffle,
    const std::vector<double>& enduranceSec,
    const std::vector<int>& taskTypes,
    const std::vector<int>& taskPriorities,
    double priPenaltyCoeff,
    const std::vector<double>& priPenaltyCurve,
    const std::vector<double>& taskArriveSec,
    double baseNowSec,
    double waitPenaltyCoeff
) {
    (void)banShuffle;
    auto states = opShift(best, SE, alpha, rng);
    auto [newBest, fNew] = selectBest(states, amrNum, taskNum, enduranceSec, taskTypes,
                                      taskPriorities, priPenaltyCoeff, priPenaltyCurve,
                                      taskArriveSec, baseNowSec, waitPenaltyCoeff);
    if (fNew < fBest) return {newBest, fNew};
    return {best, fBest};
}

// 一次 sym 变换（等价于 sym1.m），若改善则进行一次 op_shuffle 探索
std::pair<std::vector<int>, double> symOnce(
    const std::vector<int>& best, double fBest, int alpha, int SE,
    int amrNum, int taskNum, std::mt19937& rng, bool banShuffle,
    const std::vector<double>& enduranceSec,
    const std::vector<int>& taskTypes,
    const std::vector<int>& taskPriorities,
    double priPenaltyCoeff,
    const std::vector<double>& priPenaltyCurve,
    const std::vector<double>& taskArriveSec,
    double baseNowSec,
    double waitPenaltyCoeff
) {
    auto states = opSym(best, SE, alpha, rng);
    auto [newBest, fNew] = selectBest(states, amrNum, taskNum, enduranceSec, taskTypes,
                                      taskPriorities, priPenaltyCoeff, priPenaltyCurve,
                                      taskArriveSec, baseNowSec, waitPenaltyCoeff);
    if (fNew < fBest) {
        // optional shuffle stage
        if (!banShuffle) {
            auto states2 = opShuffle(best, newBest, SE, rng);
            auto [newBest2, fNew2] = selectBest(states2, amrNum, taskNum, enduranceSec, taskTypes,
                                               taskPriorities, priPenaltyCoeff, priPenaltyCurve,
                                               taskArriveSec, baseNowSec, waitPenaltyCoeff);
            if (fNew2 < fNew) return {newBest2, fNew2};
        }
        return {newBest, fNew};
    }
    return {best, fBest};
}

// 检查指定 AMR 序列是否会被 detour/时间阈值截断（true 表示安全）
static bool isSequenceValidNoDetourCut(
    const std::vector<std::vector<int>>& amrTasks,
    int amrIdx,
    int taskNum,
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList
) {
    if (amrIdx < 0 || amrIdx >= (int)amrTasks.size()) return false;
    const auto& seq = amrTasks[amrIdx];
    int maxAheadMs = amrList[amrIdx].getMaxTimeAheadMs();
    int maxDetourMm = amrList[amrIdx].getMaxDetourDistanceMm();
    if (maxAheadMs <= 0 && maxDetourMm <= 0) return true;
    if (seq.empty()) return true;

    double t = 0.0;
    double cumDetour = 0.0;
    for (size_t pos = 0; pos < seq.size(); ++pos) {
        if (pos > 0 && maxDetourMm > 0) {
            int prevIdx0 = seq[pos-1];
            int curIdx0  = seq[pos];
            double dmm = TaskAllocationUtils::minkowskiDistanceBetweenTasks(taskList[prevIdx0], taskList[curIdx0]);
            if (dmm > 0) {
                cumDetour += dmm;
                if (cumDetour >= (double)maxDetourMm) return false;
            }
        }
        if (maxAheadMs > 0) {
            double startTime = 0.0;
            if (pos == 0) {
                double c = TaskAllocationUtils::getCost(taskNum, seq[0], amrIdx);
                if (!(c < std::numeric_limits<double>::infinity())) return false;
                startTime = c; t = c;
            } else {
                startTime = t;
            }
            if (startTime >= static_cast<double>(maxAheadMs)) return false;
            if (pos < seq.size() - 1) {
                double c2 = TaskAllocationUtils::getCost(seq[pos], seq[pos+1], amrIdx);
                if (!(c2 < std::numeric_limits<double>::infinity())) return false;
                t = startTime + c2;
            }
        }
    }
    return true;
}

} // namespace

// -------------------------- PostaTaskAllocator 实现 --------------------------

PostaTaskAllocator::PostaTaskAllocator()
    : initialCode_(std::nullopt), timeLimitSec_(0.2), SE_(30), banShuffle_(false),
      priorityPenaltyCoeff_(0.02),
      priorityPenaltyCurve_({
        0.0, // 占位0
        0.4, 0.5, 0.7, 0.9, 1.2,
        1.6, 2.1, 2.7, 3.4, 4.2
      }) {}

void PostaTaskAllocator::setInitialSolution(const std::vector<int>& code) {
    initialCode_ = code;
}

void PostaTaskAllocator::setTimeLimit(double seconds) { timeLimitSec_ = seconds; }
void PostaTaskAllocator::setCandidateCount(int se) { SE_ = se; }
void PostaTaskAllocator::setBanShuffle(bool ban) { banShuffle_ = ban; }
void PostaTaskAllocator::setRandomSeed(uint32_t seed) { fixedSeed_ = seed; }

AllocationResult PostaTaskAllocator::allocate(
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList
) const {
    const int amrNum = static_cast<int>(amrList.size());
    const int taskNum = static_cast<int>(taskList.size());
    // 固定随机种子（可用 ALLOC_SEED 覆盖），便于调试复现
    uint32_t seed = fixedSeed_.value_or(1337u);
    if (!fixedSeed_.has_value()) {
        if (const char* ev = std::getenv("ALLOC_SEED")) {
            try { seed = static_cast<uint32_t>(std::stoul(ev)); } catch(...) {}
        }
    }
    std::mt19937 rng(seed);

    // 初始化解：使用外部提供的初始解或随机初始化
    std::vector<int> best;
    if (initialCode_.has_value() && !initialCode_->empty()) {
        best = normalizeInitialCode(*initialCode_, taskNum);
    } else {
        best = initializationTa(taskNum, amrNum, rng);
    }
    // 准备续航与任务类型
    std::vector<double> enduranceSec(amrNum, 1e18);
    for (int i=0;i<amrNum;++i) {
        double e = amrList[i].getEndurance();
        enduranceSec[i] = (e > 0 ? e * 3600.0 * 1000.0 : 1e18);
    }
    std::vector<int> taskTypes(taskNum, 0);
    for (int i=0;i<taskNum;++i) taskTypes[i] = taskList[i].getTaskType();
    std::vector<int> taskPriorities(taskNum, 0);
    for (int i=0;i<taskNum;++i) taskPriorities[i] = taskList[i].getPriority();
    // 任务到达时间（优先取 createTimestampISO，其次 expectedStartTimeISO）
    std::vector<double> taskArriveSec(taskNum, -1.0);
    for (int i = 0; i < taskNum; ++i) {
        const auto& t = taskList[i];
        const std::string& ts = !t.getCreateTimestampISO().empty()
            ? t.getCreateTimestampISO()
            : t.getExpectedStartTimeISO();
        if (!ts.empty()) {
            double sec = 0.0;
            if (TaskAllocationUtils::parseIso8601ToUnixSeconds(ts, sec)) {
                taskArriveSec[i] = sec;
            }
        }
    }
    const double baseNowSec = TaskAllocationUtils::nowUnixSeconds();
    double waitCoeffLocal = 0.0;
    if (const char* ev = std::getenv("ALLOC_WAIT_PENALTY_COEFF")) {
        try { waitCoeffLocal = std::stod(ev); } catch(...) {}
    }

    // 若初始解缺少任务（如 Greedy 未分配部分），按“遍历所有位置取最优”的规则补齐
    // 无合法位置的任务记录到 missingUnalloc，后续并入结果未分配池
    std::vector<int> missingUnalloc;
    {
        auto present = std::vector<char>(taskNum, 0);
        auto amrTasks = TaskAllocationUtils::parseSolution(best);
        for (const auto& seq : amrTasks) for (int t : seq) if (t>=0 && t<taskNum) present[t] = 1;
        for (int miss = 0; miss < taskNum; ++miss) {
            if (present[miss]) continue;
            bool found = false;
            double bestLocalF = std::numeric_limits<double>::infinity();
            std::vector<std::vector<int>> bestLocalTasks = amrTasks;
            for (int a=0; a<amrNum; ++a) {
                size_t L = amrTasks[a].size();
                for (size_t pos=0; pos<=L; ++pos) {
                    auto tryTasks = amrTasks;
                    tryTasks[a].insert(tryTasks[a].begin() + static_cast<long>(pos), miss);
                    if (!isSequenceValidNoDetourCut(tryTasks, a, taskNum, amrList, taskList)) continue;
                    auto codeTry = TaskAllocationUtils::deParseSolution(tryTasks);
                    double fTry = evaluateCost(amrNum, taskNum, codeTry, enduranceSec, taskTypes,
                                               taskPriorities, priorityPenaltyCoeff_, priorityPenaltyCurve_,
                                               taskArriveSec, baseNowSec, waitCoeffLocal);
                    if (fTry < bestLocalF) { bestLocalF = fTry; bestLocalTasks = std::move(tryTasks); found = true; }
                }
            }
            if (found) {
                amrTasks = std::move(bestLocalTasks);
                best = TaskAllocationUtils::deParseSolution(amrTasks);
                present[miss] = 1;
            } else {
                missingUnalloc.push_back(miss);
            }
        }
    }

    // 读取可调参数
    int seLocal = SE_;
    if (const char* ev = std::getenv("ALLOC_POSTA_SE")) { try { seLocal = std::max(1, std::stoi(ev)); } catch(...) {} }
    int sweeps = 10;
    if (const char* ev = std::getenv("ALLOC_POSTA_SWEEPS")) { try { sweeps = std::max(1, std::stoi(ev)); } catch(...) {} }
    double priCoeffLocal = priorityPenaltyCoeff_;
    if (const char* ev = std::getenv("ALLOC_PRI_COEFF")) { try { priCoeffLocal = std::stod(ev); } catch(...) {} }

    double fBest = evaluateCost(amrNum, taskNum, best, enduranceSec, taskTypes,
                                taskPriorities, priCoeffLocal, priorityPenaltyCurve_,
                                taskArriveSec, baseNowSec, waitCoeffLocal);

    // 参数设置
    const std::vector<int> Omega = {25, 20, 15, 10, 5, 1};
    const int maxIter = 1000;
    const int maxNoImprove = 50;
    int noImprove = 0;
    double last = std::numeric_limits<double>::infinity();

    auto tStart = std::chrono::steady_clock::now();
    for (int iter = 0; iter < maxIter; ++iter) {
        auto tNow = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(tNow - tStart).count();
        if (elapsed > timeLimitSec_) break;

        // swapX
        auto [best1, f1, alpha1] = updateAlpha(best, fBest, Omega, seLocal, amrNum, taskNum, rng,
                                               enduranceSec, taskTypes, taskPriorities,
                                               priCoeffLocal, priorityPenaltyCurve_,
                                               taskArriveSec, baseNowSec, waitCoeffLocal);
        best = best1; fBest = f1;
        for (int k = 0; k < sweeps; ++k) {
            auto p = swapOnce(best, fBest, alpha1, seLocal, amrNum, taskNum, rng, banShuffle_,
                              enduranceSec, taskTypes, taskPriorities,
                              priCoeffLocal, priorityPenaltyCurve_,
                              taskArriveSec, baseNowSec, waitCoeffLocal);
            best = p.first; fBest = p.second;
        }
        // shiftX
        auto [best2, f2, alpha2] = updateAlpha(best, fBest, Omega, seLocal, amrNum, taskNum, rng,
                                               enduranceSec, taskTypes, taskPriorities,
                                               priCoeffLocal, priorityPenaltyCurve_,
                                               taskArriveSec, baseNowSec, waitCoeffLocal);
        best = best2; fBest = f2;
        for (int k = 0; k < sweeps; ++k) {
            auto p = shiftOnce(best, fBest, alpha2, seLocal, amrNum, taskNum, rng, banShuffle_,
                               enduranceSec, taskTypes, taskPriorities,
                               priCoeffLocal, priorityPenaltyCurve_,
                               taskArriveSec, baseNowSec, waitCoeffLocal);
            best = p.first; fBest = p.second;
        }
        // symX
        auto [best3, f3, alpha3] = updateAlpha(best, fBest, Omega, seLocal, amrNum, taskNum, rng,
                                               enduranceSec, taskTypes, taskPriorities,
                                               priCoeffLocal, priorityPenaltyCurve_,
                                               taskArriveSec, baseNowSec, waitCoeffLocal);
        best = best3; fBest = f3;
        for (int k = 0; k < sweeps; ++k) {
            auto p = symOnce(best, fBest, alpha3, seLocal, amrNum, taskNum, rng, banShuffle_,
                             enduranceSec, taskTypes, taskPriorities,
                             priCoeffLocal, priorityPenaltyCurve_,
                             taskArriveSec, baseNowSec, waitCoeffLocal);
            best = p.first; fBest = p.second;
        }

        // no extra balancing; keep core POSTA operators only

        if (fBest < last) noImprove = 0; else if (++noImprove >= maxNoImprove) break;
        last = fBest;
    }

    // 组装结果
    AllocationResult result;
    result.code = best;
    result.amrTasks = TaskAllocationUtils::parseSolution(best);
    result.totalCost = TaskAllocationUtils::calculateTotalCost(amrNum, taskNum, best);

    // 修复无效任务（沿用基类实现）
    result = repairInvalidDuties(result, amrList, taskList);
    // 合并补齐阶段未能插入的任务到未分配池（避免重复）
    if (!missingUnalloc.empty()) {
        std::vector<char> seen(taskNum, 0);
        for (int t : result.unallocatedTaskIds) if (t>=0 && t<taskNum) seen[t] = 1;
        for (int t : missingUnalloc) if (t>=0 && t<taskNum && !seen[t]) result.unallocatedTaskIds.push_back(t);
    }
    return result;
}
