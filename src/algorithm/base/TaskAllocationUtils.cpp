#include "algorithm/base/TaskAllocationUtils.h"
#include "data/MapInfo.h"

#include <algorithm>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <cmath>
#include <cctype>
#include <unordered_map>

static constexpr double kMsPerSec = 1000.0;

// 解析编码向量（kCodeSeparator 为分隔符，任务ID为0开始）
std::vector<std::vector<int>> TaskAllocationUtils::parseSolution(const std::vector<int>& code) {
    std::vector<std::vector<int>> amrTasks;
    std::vector<int> currentTasks;

    for (int val : code) {
        if (val == kCodeSeparator) {
            amrTasks.push_back(currentTasks);
            currentTasks.clear();
        } else {
            currentTasks.push_back(val);
        }
    }
    amrTasks.push_back(currentTasks);  // 最后一个AMR的任务
    return amrTasks;
}

// 编码任务列表（kCodeSeparator 为分隔符，任务ID保持0开始）
std::vector<int> TaskAllocationUtils::deParseSolution(const std::vector<std::vector<int>>& amrTasks) {
    std::vector<int> code;
    for (size_t i = 0; i < amrTasks.size(); ++i) {
        for (int task : amrTasks[i]) {
            code.push_back(task);
        }
        if (i != amrTasks.size() - 1) {
            code.push_back(kCodeSeparator);  // 分隔符（最后一个AMR后不加）
        }
    }
    return code;
}

static double g_speed_mmps = 1000.0;
static const MapInfo* g_map_info = nullptr;

// 稀疏 Cost 缓存（可选启用）：第一阶段仅作为读适配，生成器会填充写入，读侧优先从此处获取
static bool g_sparse_enabled = false;
static int g_sparse_task_num = 0;
static int g_sparse_amr_num = 0;
static std::unordered_map<long long, double> g_sparse_cost; // key(prevOrTaskNum,curIdx,amrIdx)

static inline long long make_key_sparse(int prevOrTaskNum, int curIdx, int amrIdx) {
    // 64bit 简单哈希组合，prev/cur/amr 假设都在 0..几千范围内
    return ( (static_cast<long long>(prevOrTaskNum) << 42) ^
             (static_cast<long long>(curIdx)        << 21) ^
             (static_cast<long long>(amrIdx)) );
}

// 计算总成本
double TaskAllocationUtils::calculateTotalCost(
    int amrNum,
    int taskNum,
    const std::vector<int>& code
) {
    double totalCost = 0.0;
    double currentAmrCost = 0.0;
    double maxAmrCost = 0.0;
    int currentAmrId = 0;  // 当前AMR索引（0开始）
    int prevTask = -1;     // 上一个任务（-1表示无前置任务）
    const double INF = std::numeric_limits<double>::infinity();

    for (int val : code) {
        if (val == kCodeSeparator) {
            // 切换AMR：更新最大成本，重置当前状态
            maxAmrCost = std::max(maxAmrCost, currentAmrCost);
            currentAmrCost = 0.0;
            prevTask = -1;
            if (currentAmrId + 1 < amrNum) ++currentAmrId;
        } else {
            int currentTask = val;
            double costVal = INF;
            // 检查类型匹配并计算成本
            if (prevTask == -1) {
                costVal = TaskAllocationUtils::getCost(taskNum, currentTask, currentAmrId);
            } else {
                costVal = TaskAllocationUtils::getCost(prevTask, currentTask, currentAmrId);
            }
            // 保留原始INF语义：若某段不可达，则总成本为INF，便于上层识别问题
            if (!std::isfinite(costVal)) return std::numeric_limits<double>::infinity();
            totalCost += costVal;
            currentAmrCost += costVal;
            prevTask = currentTask;
        }
    }
    // 处理最后一个AMR的成本
    maxAmrCost = std::max(maxAmrCost, currentAmrCost);
    // 目标函数：平均成本 + 最大成本
    double avgCost = totalCost / std::max(1, amrNum);
    double totalMs = avgCost + maxAmrCost;
    return totalMs / kMsPerSec;
}

// 计算每个AMR的累计时间（毫秒）
std::vector<double> TaskAllocationUtils::computeAmrDurations(
    int amrNum,
    int taskNum,
    const std::vector<int>& code
) {
    std::vector<double> amrTime(amrNum, 0.0);
    int currentAmrId = 0;
    int prevTask = -1;
    for (int val : code) {
        if (val == kCodeSeparator) {
            currentAmrId = std::min(currentAmrId + 1, amrNum - 1);
            prevTask = -1;
            continue;
        }
        int currentTask = val;
        double costVal = 0.0;
        if (prevTask == -1) costVal = TaskAllocationUtils::getCost(taskNum, currentTask, currentAmrId);
        else costVal = TaskAllocationUtils::getCost(prevTask, currentTask, currentAmrId);
        if (!std::isfinite(costVal)) return amrTime; // 若不可达，直接返回当前累计，避免误判
        amrTime[currentAmrId] += costVal;
        prevTask = currentTask;
    }
    return amrTime;
}

void TaskAllocationUtils::setGlobalSpeedMmPerSec(double v) { g_speed_mmps = v; }
double TaskAllocationUtils::getGlobalSpeedMmPerSec() { return g_speed_mmps; }
void TaskAllocationUtils::setMapInfoPtr(const MapInfo* m) { g_map_info = m; }
const MapInfo* TaskAllocationUtils::getMapInfoPtr() { return g_map_info; }
double TaskAllocationUtils::minkowskiDistanceBetweenTasks(const Task& a, const Task& b, double p) {
    if (!g_map_info) return -1.0;
    int ida = a.getStartId();
    int idb = b.getStartId();
    try {
        const Node& na = g_map_info->getNodeById(ida);
        const Node& nb = g_map_info->getNodeById(idb);
    double dx = std::abs(na.x - nb.x); double dy = std::abs(na.y - nb.y);
    if (p <= 1.0) return dx + dy; // 默认曼哈顿
    return std::pow(std::pow(dx, p) + std::pow(dy, p), 1.0 / p);
    } catch(...) { return -1.0; }
}

void TaskAllocationUtils::sparseReset(int taskNum, int amrNum) {
    g_sparse_cost.clear();
    g_sparse_task_num = taskNum;
    g_sparse_amr_num = amrNum;
    // 稀疏-only 默认开启；若明确设置 ALLOC_COST_SPARSE=0/false 才关闭
    g_sparse_enabled = true;
    if (const char* ev = std::getenv("ALLOC_COST_SPARSE")) {
        std::string v(ev);
        if (v=="0" || v=="false" || v=="False" || v=="FALSE") g_sparse_enabled = false;
    }
}

void TaskAllocationUtils::sparsePut(int prevOrTaskNum, int curIdx, int amrIdx, double value) {
    long long k = make_key_sparse(prevOrTaskNum, curIdx, amrIdx);
    g_sparse_cost[k] = value;
}

double TaskAllocationUtils::getCost(int prevOrTaskNum, int curIdx, int amrIdx) {
    const double INF = std::numeric_limits<double>::infinity();
    auto it = g_sparse_cost.find(make_key_sparse(prevOrTaskNum, curIdx, amrIdx));
    if (it != g_sparse_cost.end()) return it->second;
    return INF;
}

// === 时间戳相关实现 ===
double TaskAllocationUtils::nowUnixSeconds() {
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    auto secs = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto subsecs = now - secs;
    double frac = std::chrono::duration<double>(subsecs).count();
    return static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(secs.time_since_epoch()).count()) + frac;
}

static inline bool parse_int(const std::string& s, size_t pos, size_t len, int& out) {
    if (pos + len > s.size()) return false;
    int v = 0; bool neg = false; size_t i = pos;
    if (len > 0 && s[i] == '+') { i++; len--; }
    else if (len > 0 && s[i] == '-') { neg = true; i++; len--; }
    if (len == 0) return false;
    for (size_t k = 0; k < len; ++k, ++i) {
        char c = s[i]; if (!std::isdigit(static_cast<unsigned char>(c))) return false;
        v = v * 10 + (c - '0');
    }
    out = neg ? -v : v; return true;
}

bool TaskAllocationUtils::parseIso8601ToUnixSeconds(const std::string& iso, double& outSec) {
    // Minimal ISO8601 parser covering: YYYY-MM-DDTHH:MM:SS[.sss][Z|±HH:MM|±HHMM|±HH]
    // Also tolerate ' ' in place of 'T'.
    if (iso.empty()) return false;
    std::string s = iso;
    // Trim spaces
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    if (s.size() < 19) return false; // at least YYYY-MM-DDTHH:MM:SS

    // Replace space with T for uniform handling
    if (s[10] == ' ') s[10] = 'T';

    int year, mon, mday, hour, min, sec;
    if (!(parse_int(s, 0, 4, year) && s[4] == '-' && parse_int(s, 5, 2, mon) && s[7] == '-' &&
          parse_int(s, 8, 2, mday) && s[10] == 'T' && parse_int(s, 11, 2, hour) && s[13] == ':' &&
          parse_int(s, 14, 2, min) && s[16] == ':' && parse_int(s, 17, 2, sec))) {
        return false;
    }

    double frac = 0.0; size_t pos = 19;
    if (pos < s.size() && s[pos] == '.') {
        // read fractional seconds (up to milliseconds/microseconds)
        size_t i = pos + 1; size_t beg = i; int digits = 0; int value = 0; int scale = 1;
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i])) && digits < 9) {
            value = value * 10 + (s[i] - '0');
            scale *= 10; ++i; ++digits;
        }
        if (i == beg) return false; // dot with no digits
        frac = static_cast<double>(value) / static_cast<double>(scale);
        pos = i;
    }

    // Timezone: Z or ±HH[:MM|MM]
    int tzOffsetSec = 0; // seconds to add to local parsed to get UTC
    if (pos < s.size()) {
        char c = s[pos];
        if (c == 'Z' || c == 'z') {
            tzOffsetSec = 0; pos += 1;
        } else if (c == '+' || c == '-') {
            int sign = (c == '+') ? 1 : -1; pos += 1;
            int th = 0, tm = 0; bool ok = false;
            if (pos + 2 <= s.size() && std::isdigit(static_cast<unsigned char>(s[pos])) &&
                std::isdigit(static_cast<unsigned char>(s[pos+1]))) {
                th = (s[pos]-'0')*10 + (s[pos+1]-'0'); pos += 2; ok = true;
                if (pos < s.size() && s[pos] == ':') {
                    pos += 1;
                    if (pos + 2 <= s.size() && std::isdigit(static_cast<unsigned char>(s[pos])) &&
                        std::isdigit(static_cast<unsigned char>(s[pos+1]))) {
                        tm = (s[pos]-'0')*10 + (s[pos+1]-'0'); pos += 2;
                    } else ok = false;
                } else if (pos + 2 <= s.size() && std::isdigit(static_cast<unsigned char>(s[pos])) &&
                           std::isdigit(static_cast<unsigned char>(s[pos+1]))) {
                    tm = (s[pos]-'0')*10 + (s[pos+1]-'0'); pos += 2; // ±HHMM
                } else {
                    // only ±HH
                }
            }
            if (!ok) return false;
            tzOffsetSec = sign * (th * 3600 + tm * 60);
        } else {
            // Unknown trailing characters → reject
            // allow whitespace only
            while (pos < s.size()) {
                if (!std::isspace(static_cast<unsigned char>(s[pos]))) return false;
                ++pos;
            }
        }
    }

    std::tm tm{}; tm.tm_year = year - 1900; tm.tm_mon = mon - 1; tm.tm_mday = mday; tm.tm_hour = hour; tm.tm_min = min; tm.tm_sec = sec; tm.tm_isdst = -1;

    // Convert to time_t in UTC
    time_t t_utc;
#if defined(_WIN32)
    t_utc = _mkgmtime(&tm);
#else
    t_utc = timegm(&tm);
#endif
    if (t_utc == static_cast<time_t>(-1)) return false;

    // If timezone offset is present and not Z, adjust: e.g. 10:00+08:00 means local=UTC+8 → UTC = local - 8h
    // Here we parsed components as UTC clock; we need to subtract offset to obtain UTC epoch.
    double base = static_cast<double>(t_utc) - static_cast<double>(tzOffsetSec);
    outSec = base + frac;
    return true;
}

double TaskAllocationUtils::getMaxTimestampSkewSec(double defaultSec) {
    auto read_env = [](const char* k)->const char* { const char* v = std::getenv(k); return v && v[0]? v : nullptr; };
    const char* v = nullptr;
    if (!v) v = read_env("TS_MAX_SKEW_SEC");
    if (!v) v = read_env("ALLOC_TS_MAX_SKEW_SEC");
    if (!v) v = read_env("EXT_TS_MAX_SKEW_SEC");
    if (!v) return defaultSec;
    char* end = nullptr; double d = std::strtod(v, &end);
    if (end == v || !std::isfinite(d) || d < 0) return defaultSec;
    return d;
}

bool TaskAllocationUtils::isTimestampTrusted(const std::string& isoTs,
                                             double maxSkewSec,
                                             double* outAbsDeltaSec) {
    double msgSec = 0.0;
    if (!parseIso8601ToUnixSeconds(isoTs, msgSec)) return false;
    double nowSec = nowUnixSeconds();
    double d = std::fabs(nowSec - msgSec);
    if (outAbsDeltaSec) *outAbsDeltaSec = d;
    return d <= maxSkewSec;
}

// (moved above) 全局速度由 g_speed_mmps 持有
