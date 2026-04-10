#include "algorithm/CostMatrixGenerator.h"
#include "common/AmrPositionResolver.h"
#include "common/CongestionRegionUtils.h"
#include "data/MapInfo.h"
#include "algorithm/base/TaskAllocationUtils.h"
#include <limits>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include "common/TaskPolicy.h"
// #include <queue> // not needed (sync Dijkstra removed)

// CostMatrixGenerator.cpp 核心修改

static constexpr double kMsPerSec = 1000.0;

static std::string getenv_str(const char* key, const char* defv) {
    const char* v = std::getenv(key);
    if (v && v[0]) return std::string(v);
    return defv ? std::string(defv) : std::string();
}

static std::string to_lower_ascii(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
}

// 任务总时长 = 所有子任务 estimatedDuration 之和（毫秒）
static double computeTaskDurationMs(const Task& task) {
    std::int64_t sum = 0;
    const auto& subs = task.getSubTasks();
    for (const auto& st : subs) {
        sum += std::max<std::int64_t>(0, st.getEstimatedDurationMs());
    }
    return static_cast<double>(sum);
}

static bool compute_map_bounds(const MapInfo& mapInfo,
                               double& minX,
                               double& maxX,
                               double& minY,
                               double& maxY) {
    const auto& nodes = mapInfo.getNodes();
    bool has = false;
    for (const auto& node : nodes) {
        if (node.id < 0) continue;
        if (!has) {
            minX = maxX = node.x;
            minY = maxY = node.y;
            has = true;
            continue;
        }
        minX = std::min(minX, node.x);
        maxX = std::max(maxX, node.x);
        minY = std::min(minY, node.y);
        maxY = std::max(maxY, node.y);
    }
    return has;
}

static int auto_region_grid(int amrCount, int minGrid, int maxGrid) {
    return CongestionRegionUtils::autoRegionGrid(amrCount, minGrid, maxGrid);
}

static bool is_passable_region_node(const Node& node) {
    return CongestionRegionUtils::isPassableRegionNode(node);
}

static void gather_neighbors_indices(const MapInfo& mapInfo,
                                     int idx,
                                     std::vector<int>& out) {
    CongestionRegionUtils::gatherNeighborIndices(mapInfo, idx, out);
}

static int build_graph_regions(const MapInfo& mapInfo,
                               int targetRegions,
                               std::vector<int>& regionByIndex,
                               std::vector<int>& seedIndices,
                               std::vector<char>* bridgeRegionMask,
                               bool enableBridgeRegions) {
    return CongestionRegionUtils::buildGraphRegions(
        mapInfo,
        targetRegions,
        regionByIndex,
        seedIndices,
        bridgeRegionMask,
        enableBridgeRegions);
}

// NOTE: synchronous Dijkstra helper removed. All SP resolved via ShortestPathUpdater.
static std::vector<std::vector<std::vector<double>>> generateCostMatrixImpl(
    const std::vector<Amr>& amrList,
    const std::vector<Task>& taskList,
    const ShortestPathUpdater& shortestPathUpdater,
    const MapInfo& mapInfo,
    const std::vector<int>* startNodeOverride,
    const std::vector<double>* startTimeOffsetMs
) {
    int amrNum = amrList.size();    // AMR列表长度（索引0~amrNum-1）
    int taskNum = taskList.size();  // 任务列表长度（索引0~taskNum-1）
    const double INF = std::numeric_limits<double>::infinity();
    auto getenv_int = [](const char* k, int d){ const char* v = std::getenv(k); if (!v) return d; try { return std::stoi(v);} catch(...) { return d; } };
    auto getenv_double = [](const char* k, double d){ const char* v = std::getenv(k); if (!v) return d; try { return std::stod(v);} catch(...) { return d; } };
    // Top-K 控制：默认取比例 ALLOC_TOPK_RATIO（默认0.5）× taskNum，可被环境变量覆盖
    double ratio = getenv_double("ALLOC_TOPK_RATIO", 0.5);
    if (!(ratio > 0.0)) ratio = 0.5;
    int defaultTopK = std::max(1, (int)std::lround(ratio * taskNum));
    int topKAgent = getenv_int("ALLOC_TOPK_AGENT", getenv_int("ALLOC_TOPK", defaultTopK));
    int topKNext  = getenv_int("ALLOC_TOPK_NEXT",  getenv_int("ALLOC_TOPK", defaultTopK));
    topKAgent = std::min(std::max(1, topKAgent), taskNum);
    topKNext  = std::min(std::max(1, topKNext),  taskNum);

    // 针对性充电任务：若某AMR在任务列表中存在专门指派的充电任务（taskType==3且deviceIds包含该AMR），
    // 则该AMR对非充电任务的成本强制为INF（不赋值）
    std::vector<bool> forceCharge(amrNum, false);
    for (int amrIdx = 0; amrIdx < amrNum; ++amrIdx) {
        const std::string& did = amrList[amrIdx].getDeviceId();
        for (const auto& t : taskList) {
            if (t.getTaskType() == 3) {
                for (const auto& id : t.getDeviceIds()) {
                    if (id == did) { forceCharge[amrIdx] = true; break; }
                }
            }
            if (forceCharge[amrIdx]) break;
        }
    }

    // 预取任务起点/终点ID与坐标（用于Top-K启发选择）并预计算服务时长
    std::vector<int> taskStartIdList(taskNum, -1);
    std::vector<int> taskEndIdList(taskNum, -1);
    std::vector<std::pair<double,double>> taskStartXY(taskNum, {0.0, 0.0});
    std::vector<double> taskSvcMs(taskNum, 0.0);
    for (int i=0;i<taskNum;++i) {
        auto first_valid_node = [](const std::vector<SubTask>& subs) {
            for (const auto& st : subs) {
                int nid = st.getPoint().getNodeId();
                if (nid >= 0) return nid;
            }
            return -1;
        };
        auto last_valid_node = [](const std::vector<SubTask>& subs) {
            for (auto it = subs.rbegin(); it != subs.rend(); ++it) {
                int nid = it->getPoint().getNodeId();
                if (nid >= 0) return nid;
            }
            return -1;
        };
        int sid = taskList[i].getStartId();
        int eid = taskList[i].getEndId();
        const auto& subsTmp = taskList[i].getSubTasks();
        if (!subsTmp.empty()) {
            int firstNode = first_valid_node(subsTmp);
            int lastNode = last_valid_node(subsTmp);
            if (firstNode >= 0) sid = firstNode;
            if (lastNode >= 0) eid = lastNode;
        }
        if (sid < 0 && eid >= 0) sid = eid;
        if (eid < 0 && sid >= 0) eid = sid;
        try {
            const Node& n = mapInfo.getNodeById(sid);
            taskStartIdList[i] = sid;
            taskStartXY[i] = {n.x, n.y};
        } catch(...) {
            taskStartIdList[i] = -1;
        }
        taskEndIdList[i] = eid;
        taskSvcMs[i] = computeTaskDurationMs(taskList[i]);
    }

    // 通知稀疏缓存：本轮任务与AMR维度（读端默认使用稀疏）
    TaskAllocationUtils::sparseReset(taskNum, amrNum);
    // 稀疏-only：不再分配/写入 dense 三维矩阵
    std::vector<std::vector<std::vector<double>>> costMatrix;

    // 1. 生成 agent→task 首段成本，直接写入 costMatrix[taskNum][task][amr]
    double globalSpeed = mapInfo.getGlobalMaxSpeed();
    if (globalSpeed <= 0) globalSpeed = 1000.0; // 兜底速度（mm/s）
    // 将构建时使用的速度提供给后续修复阶段用于距离估算，并提供地图用于几何距离
    TaskAllocationUtils::setGlobalSpeedMmPerSec(globalSpeed);
    TaskAllocationUtils::setMapInfoPtr(&mapInfo);

    // 记录AMR的起点ID与类型，便于覆盖纠偏阶段使用
    std::vector<int> amrStartId(amrNum, -1);
    std::vector<int> amrTypeIndexVec(amrNum, 0);
    for (int amrIdx = 0; amrIdx < amrNum; ++amrIdx) {  // amrIdx是amrList的索引
        const Amr& amr = amrList[amrIdx];
        int amrType = amr.getDeviceType();
        int node_id = -1;
        if (startNodeOverride && (int)startNodeOverride->size()==amrNum && (*startNodeOverride)[amrIdx] != -1) {
            node_id = (*startNodeOverride)[amrIdx];
        } else {
            // 统一AMR起点解析逻辑
            node_id = AmrPositionResolver::resolveStartNodeId(amr, mapInfo);
        }
        // 验证节点ID有效
        try { (void)mapInfo.getNodeById(node_id); } catch(...) { continue; }
        amrStartId[amrIdx] = node_id;
        amrTypeIndexVec[amrIdx] = amrType;
    }

    double nowSec = TaskAllocationUtils::nowUnixSeconds();
    double ageThresholdSec = getenv_double("ALLOC_TASK_AGE_THRESHOLD_SEC", 300.0);
    double ageBonusMsPerSec = getenv_double("ALLOC_TASK_AGE_BONUS_MS_PER_SEC", 100.0);
    double ageBonusMaxMs = getenv_double("ALLOC_TASK_AGE_BONUS_MAX_MS", 120000.0);
    double ageMinCostMs = getenv_double("ALLOC_TASK_AGE_MIN_COST_MS", -60000.0);
    if (!std::isfinite(ageMinCostMs)) ageMinCostMs = -60000.0;
    std::vector<double> taskAgeBonusMs(taskNum, 0.0);
    if (ageBonusMsPerSec > 0.0 && ageThresholdSec >= 0.0) {
        for (int i = 0; i < taskNum; ++i) {
            const Task& task = taskList[i];
            std::string ts = task.getCreateTimestampISO();
            if (ts.empty()) ts = task.getExpectedStartTimeISO();
            double tsSec = 0.0;
            if (ts.empty() || !TaskAllocationUtils::parseIso8601ToUnixSeconds(ts, tsSec)) continue;
            double ageSec = nowSec - tsSec;
            if (ageSec <= ageThresholdSec) continue;
            double overSec = ageSec - ageThresholdSec;
            double bonus = overSec * ageBonusMsPerSec;
            if (ageBonusMaxMs > 0.0) bonus = std::min(bonus, ageBonusMaxMs);
            if (bonus > 0.0) taskAgeBonusMs[i] = bonus;
        }
    }

    double congestionPenaltyRatio = getenv_double("ALLOC_CONGESTION_PENALTY_RATIO", 0.5);
    std::vector<double> taskCongestionScore(taskNum, 0.0);
    if (congestionPenaltyRatio > 0.0 && amrNum > 0 && taskNum > 0) {
        int grid = getenv_int("ALLOC_REGION_GRID", 0);
        if (grid <= 0) grid = getenv_int("CONGESTION_REGION_GRID", 0);
        int minGrid = getenv_int("ALLOC_REGION_GRID_MIN", 0);
        if (minGrid <= 0) minGrid = getenv_int("CONGESTION_REGION_GRID_MIN", 3);
        int maxGrid = getenv_int("ALLOC_REGION_GRID_MAX", 0);
        if (maxGrid <= 0) maxGrid = getenv_int("CONGESTION_REGION_GRID_MAX", 6);
        if (grid <= 0) grid = auto_region_grid(amrNum, minGrid, maxGrid);
        int window = getenv_int("ALLOC_REGION_WINDOW", 0);
        if (window <= 0) window = getenv_int("CONGESTION_REGION_WINDOW", 2);
        if (window < 0) window = 0;
        std::string mode = to_lower_ascii(getenv_str("ALLOC_REGION_MODE", ""));
        if (mode.empty()) mode = to_lower_ascii(getenv_str("CONGESTION_REGION_MODE", "graph"));
        bool useGraph = (mode == "graph" || mode == "cluster" || mode == "partition");

        if (useGraph && grid > 0) {
            int regionCount = getenv_int("ALLOC_REGION_COUNT", 0);
            if (regionCount <= 0) regionCount = getenv_int("CONGESTION_REGION_COUNT", 0);
            if (regionCount <= 0) regionCount = grid * grid;
            const bool enableBridgeRegions =
                getenv_int("ALLOC_REGION_BRIDGE_ENABLE",
                           getenv_int("CONGESTION_REGION_BRIDGE_ENABLE", 1)) != 0;
            std::vector<int> regionByIndex;
            std::vector<int> seedIndices;
            std::vector<char> bridgeRegionMask;
            regionCount = build_graph_regions(mapInfo,
                                              regionCount,
                                              regionByIndex,
                                              seedIndices,
                                              &bridgeRegionMask,
                                              enableBridgeRegions);
            if (regionCount > 0) {
                std::vector<int> regionCounts(regionCount, 0);
                const auto& id2idx = mapInfo.getId2Index();
                for (int a = 0; a < amrNum; ++a) {
                    int nodeId = amrStartId[a];
                    if (nodeId < 0) continue;
                    auto it = id2idx.find(nodeId);
                    if (it == id2idx.end()) continue;
                    int idx = it->second;
                    if (idx < 0 || idx >= static_cast<int>(regionByIndex.size())) continue;
                    int reg = regionByIndex[static_cast<size_t>(idx)];
                    if (reg >= 0 && reg < regionCount) regionCounts[reg] += 1;
                }

                const auto& nodes = mapInfo.getNodes();
                std::vector<int> regionNodeCounts(regionCount, 0);
                int totalNormalNodes = 0;
                int normalAgvCount = 0;
                for (size_t i = 0; i < nodes.size(); ++i) {
                    const Node& node = nodes[i];
                    if (!is_passable_region_node(node)) continue;
                    int reg = regionByIndex[i];
                    if (reg < 0 || reg >= regionCount) continue;
                    regionNodeCounts[reg] += 1;
                }
                int effectiveNormalRegions = 0;
                for (int r = 0; r < regionCount; ++r) {
                    if (r < static_cast<int>(bridgeRegionMask.size()) && bridgeRegionMask[static_cast<size_t>(r)]) {
                        continue;
                    }
                    if (regionNodeCounts[r] > 0) {
                        effectiveNormalRegions += 1;
                        totalNormalNodes += regionNodeCounts[r];
                    }
                    normalAgvCount += regionCounts[r];
                }
                if (effectiveNormalRegions <= 0) effectiveNormalRegions = 1;

                double avg = static_cast<double>(normalAgvCount) /
                             static_cast<double>(effectiveNormalRegions);
                int softLimit = getenv_int("ALLOC_REGION_SOFT", 0);
                if (softLimit <= 0) softLimit = getenv_int("CONGESTION_REGION_SOFT", 0);
                int hardLimit = getenv_int("ALLOC_REGION_HARD", 0);
                if (hardLimit <= 0) hardLimit = getenv_int("CONGESTION_REGION_HARD", 0);
                if (softLimit <= 0) {
                    softLimit = std::max(2, static_cast<int>(std::ceil(avg)) + 1);
                }
                if (hardLimit <= 0) {
                    int bump = std::max(1, static_cast<int>(std::ceil(avg / 2.0)));
                    hardLimit = softLimit + bump;
                }
                if (hardLimit <= softLimit) hardLimit = softLimit + 1;

                double avgNodes = static_cast<double>(totalNormalNodes) /
                                  static_cast<double>(std::max(1, effectiveNormalRegions));
                std::vector<int> regionSoft(regionCount, 0);
                std::vector<int> regionHard(regionCount, 0);
                for (int r = 0; r < regionCount; ++r) {
                    if (r < static_cast<int>(bridgeRegionMask.size()) && bridgeRegionMask[static_cast<size_t>(r)]) {
                        regionSoft[r] = 0;
                        regionHard[r] = 1;
                        continue;
                    }
                    int nodeCount = regionNodeCounts[r];
                    if (nodeCount <= 0) {
                        regionSoft[r] = 0;
                        regionHard[r] = 0;
                        continue;
                    }
                    double factor = (avgNodes > 0.0)
                        ? (static_cast<double>(nodeCount) / avgNodes)
                        : 1.0;
                    int s = std::max(1, static_cast<int>(std::lround(softLimit * factor)));
                    int h = std::max(1, static_cast<int>(std::lround(hardLimit * factor)));
                    if (h <= s) h = s + 1;
                    regionSoft[r] = s;
                    regionHard[r] = h;
                }

                std::vector<std::vector<int>> regionAdj(regionCount);
                std::vector<std::vector<char>> adjMat(regionCount, std::vector<char>(regionCount, 0));
                std::vector<int> neighbors;
                for (int idx = 0; idx < static_cast<int>(nodes.size()); ++idx) {
                    if (!is_passable_region_node(nodes[static_cast<size_t>(idx)])) continue;
                    int reg = regionByIndex[static_cast<size_t>(idx)];
                    if (reg < 0 || reg >= regionCount) continue;
                    gather_neighbors_indices(mapInfo, idx, neighbors);
                    for (int nb : neighbors) {
                        if (nb < 0 || nb >= static_cast<int>(nodes.size())) continue;
                        if (!is_passable_region_node(nodes[static_cast<size_t>(nb)])) continue;
                        int reg2 = regionByIndex[static_cast<size_t>(nb)];
                        if (reg2 < 0 || reg2 >= regionCount || reg2 == reg) continue;
                        if (!adjMat[reg][reg2]) {
                            adjMat[reg][reg2] = 1;
                            adjMat[reg2][reg] = 1;
                            regionAdj[reg].push_back(reg2);
                            regionAdj[reg2].push_back(reg);
                        }
                    }
                }

                int radius = window;
                std::vector<int> windowCounts(regionCount, 0);
                std::vector<int> windowSoft(regionCount, 0);
                std::vector<int> windowHard(regionCount, 0);

                auto accumulate_region = [&](int start,
                                             int radius,
                                             int& outCount,
                                             int& outSoft,
                                             int& outHard) {
                    outCount = 0;
                    outSoft = 0;
                    outHard = 0;
                    std::vector<int> dist(regionCount, -1);
                    std::deque<int> q;
                    dist[start] = 0;
                    q.push_back(start);
                    while (!q.empty()) {
                        int cur = q.front();
                        q.pop_front();
                        outCount += regionCounts[cur];
                        outSoft += regionSoft[cur];
                        outHard += regionHard[cur];
                        if (dist[cur] >= radius) continue;
                        for (int nb : regionAdj[cur]) {
                            if (nb < 0 || nb >= regionCount) continue;
                            if (dist[nb] >= 0) continue;
                            dist[nb] = dist[cur] + 1;
                            q.push_back(nb);
                        }
                    }
                };

                for (int r = 0; r < regionCount; ++r) {
                    int sumCount = 0;
                    int sumSoft = 0;
                    int sumHard = 0;
                    accumulate_region(r, radius, sumCount, sumSoft, sumHard);
                    if (sumHard <= sumSoft) sumHard = sumSoft + 1;
                    if (sumHard <= 0) sumHard = 1;
                    windowCounts[r] = sumCount;
                    windowSoft[r] = sumSoft;
                    windowHard[r] = sumHard;
                }

                std::vector<double> regionScore(regionCount, 0.0);
                for (int r = 0; r < regionCount; ++r) {
                    const bool isBridgeRegion =
                        (r < static_cast<int>(bridgeRegionMask.size()) &&
                         bridgeRegionMask[static_cast<size_t>(r)]);
                    int count = isBridgeRegion ? regionCounts[r] : windowCounts[r];
                    int soft = isBridgeRegion ? regionSoft[r] : windowSoft[r];
                    int hard = isBridgeRegion ? regionHard[r] : windowHard[r];
                    double score = 0.0;
                    if (count >= hard) {
                        score = 1.0;
                    } else if (count > soft && hard > soft) {
                        score = static_cast<double>(count - soft) /
                                static_cast<double>(hard - soft);
                    }
                    regionScore[r] = std::clamp(score, 0.0, 1.0);
                }

                for (int t = 0; t < taskNum; ++t) {
                    int nodeId = taskStartIdList[t];
                    if (nodeId < 0) continue;
                    auto it = id2idx.find(nodeId);
                    if (it == id2idx.end()) continue;
                    int idx = it->second;
                    if (idx < 0 || idx >= static_cast<int>(regionByIndex.size())) continue;
                    int reg = regionByIndex[static_cast<size_t>(idx)];
                    if (reg >= 0 && reg < regionCount) {
                        taskCongestionScore[t] = regionScore[reg];
                    }
                }
            }
        } else if (grid > 0) {
            double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0;
            if (compute_map_bounds(mapInfo, minX, maxX, minY, maxY)) {
                double spanX = std::max(1e-6, maxX - minX);
                double spanY = std::max(1e-6, maxY - minY);
                double cellW = spanX / static_cast<double>(grid);
                double cellH = spanY / static_cast<double>(grid);

                auto cell_index = [&](double x, double y) -> int {
                    int cx = static_cast<int>(std::floor((x - minX) / cellW));
                    int cy = static_cast<int>(std::floor((y - minY) / cellH));
                    if (cx < 0) cx = 0;
                    if (cy < 0) cy = 0;
                    if (cx >= grid) cx = grid - 1;
                    if (cy >= grid) cy = grid - 1;
                    return cy * grid + cx;
                };

                std::vector<int> cellCounts(static_cast<size_t>(grid * grid), 0);
                for (int a = 0; a < amrNum; ++a) {
                    bool hasPos = false;
                    double ax = 0.0, ay = 0.0;
                    int nodeId = amrStartId[a];
                    if (nodeId >= 0) {
                        try {
                            const Node& an = mapInfo.getNodeById(nodeId);
                            ax = an.x; ay = an.y; hasPos = true;
                        } catch (...) {
                            hasPos = false;
                        }
                    }
                    if (!hasPos) {
                        const Amr& amr = amrList[a];
                        ax = amr.getX();
                        ay = amr.getY();
                        hasPos = true;
                    }
                    if (!hasPos) continue;
                    int cell = cell_index(ax, ay);
                    if (cell >= 0) cellCounts[static_cast<size_t>(cell)] += 1;
                }

                std::vector<int> cellNodeCounts(static_cast<size_t>(grid * grid), 0);
                int totalNodes = 0;
                const auto& nodes = mapInfo.getNodes();
                for (const auto& node : nodes) {
                    if (!is_passable_region_node(node)) continue;
                    int cell = cell_index(node.x, node.y);
                    if (cell >= 0) {
                        cellNodeCounts[static_cast<size_t>(cell)] += 1;
                        totalNodes += 1;
                    }
                }
                int effectiveCells = 0;
                for (int c : cellNodeCounts) {
                    if (c > 0) effectiveCells++;
                }
                if (effectiveCells <= 0) effectiveCells = grid * grid;

                int softLimit = getenv_int("ALLOC_REGION_SOFT", 0);
                if (softLimit <= 0) softLimit = getenv_int("CONGESTION_REGION_SOFT", 0);
                int hardLimit = getenv_int("ALLOC_REGION_HARD", 0);
                if (hardLimit <= 0) hardLimit = getenv_int("CONGESTION_REGION_HARD", 0);
                double avg = static_cast<double>(amrNum) / static_cast<double>(effectiveCells);
                if (softLimit <= 0) {
                    softLimit = std::max(2, static_cast<int>(std::ceil(avg)) + 1);
                }
                if (hardLimit <= 0) {
                    int bump = std::max(1, static_cast<int>(std::ceil(avg / 2.0)));
                    hardLimit = softLimit + bump;
                }
                if (hardLimit <= softLimit) hardLimit = softLimit + 1;

                double avgNodes = static_cast<double>(totalNodes) /
                                  static_cast<double>(std::max(1, effectiveCells));
                std::vector<int> cellSoft(static_cast<size_t>(grid * grid), 0);
                std::vector<int> cellHard(static_cast<size_t>(grid * grid), 0);
                for (size_t i = 0; i < cellSoft.size(); ++i) {
                    int nodeCount = cellNodeCounts[i];
                    if (nodeCount <= 0) {
                        cellSoft[i] = 0;
                        cellHard[i] = 0;
                        continue;
                    }
                    double factor = (avgNodes > 0.0)
                        ? (static_cast<double>(nodeCount) / avgNodes)
                        : 1.0;
                    int s = std::max(1, static_cast<int>(std::lround(softLimit * factor)));
                    int h = std::max(1, static_cast<int>(std::lround(hardLimit * factor)));
                    if (h <= s) h = s + 1;
                    cellSoft[i] = s;
                    cellHard[i] = h;
                }

                std::vector<int> windowCounts(static_cast<size_t>(grid * grid), 0);
                std::vector<int> windowSoft(static_cast<size_t>(grid * grid), 0);
                std::vector<int> windowHard(static_cast<size_t>(grid * grid), 0);
                int half = window;
                for (int row = 0; row < grid; ++row) {
                    for (int col = 0; col < grid; ++col) {
                        int sumCount = 0;
                        int sumSoft = 0;
                        int sumHard = 0;
                        for (int dy = -half; dy <= half; ++dy) {
                            int ny = row + dy;
                            if (ny < 0 || ny >= grid) continue;
                            for (int dx = -half; dx <= half; ++dx) {
                                int nx = col + dx;
                                if (nx < 0 || nx >= grid) continue;
                                size_t idx = static_cast<size_t>(ny * grid + nx);
                                sumCount += cellCounts[idx];
                                sumSoft += cellSoft[idx];
                                sumHard += cellHard[idx];
                            }
                        }
                        if (sumHard <= sumSoft) sumHard = sumSoft + 1;
                        if (sumHard <= 0) sumHard = 1;
                        size_t idx = static_cast<size_t>(row * grid + col);
                        windowCounts[idx] = sumCount;
                        windowSoft[idx] = sumSoft;
                        windowHard[idx] = sumHard;
                    }
                }

                std::vector<double> cellScore(static_cast<size_t>(grid * grid), 0.0);
                for (size_t i = 0; i < cellScore.size(); ++i) {
                    int count = windowCounts[i];
                    int soft = windowSoft[i];
                    int hard = windowHard[i];
                    double score = 0.0;
                    if (count >= hard) {
                        score = 1.0;
                    } else if (count > soft && hard > soft) {
                        score = static_cast<double>(count - soft) /
                                static_cast<double>(hard - soft);
                    }
                    cellScore[i] = std::clamp(score, 0.0, 1.0);
                }

                for (int t = 0; t < taskNum; ++t) {
                    int nodeId = taskStartIdList[t];
                    if (nodeId < 0) continue;
                    try {
                        const Node& tn = mapInfo.getNodeById(nodeId);
                        int cell = cell_index(tn.x, tn.y);
                        if (cell >= 0) taskCongestionScore[t] = cellScore[static_cast<size_t>(cell)];
                    } catch (...) {
                        continue;
                    }
                }
            }
        }
    }

    auto apply_task_adjustment = [&](int taskIdx, double costMs) -> double {
        double adjusted = costMs;
        if (taskIdx >= 0 && taskIdx < taskNum) {
            double score = taskCongestionScore[taskIdx];
            if (congestionPenaltyRatio > 0.0 && score > 0.0) {
                adjusted *= (1.0 + score * congestionPenaltyRatio);
            }
            double bonus = taskAgeBonusMs[taskIdx];
            if (bonus > 0.0) {
                adjusted -= bonus;
                if (adjusted < ageMinCostMs) adjusted = ageMinCostMs;
            }
        }
        return adjusted;
    };

    // 1. 生成 agent→task 首段成本，直接写入 costMatrix[taskNum][task][amr]
    std::vector<int> amrPosIndex(amrNum, -1);
    for (int amrIdx = 0; amrIdx < amrNum; ++amrIdx) {
        const Amr& amr = amrList[amrIdx];
        int node_id = amrStartId[amrIdx];
        if (node_id < 0) continue;
        const Node& amrNode = mapInfo.getNodeById(node_id);

        // Top-K 预选：按几何近似筛选候选，随后对候选用SP求精确时间
        std::vector<std::pair<int,double>> heur; heur.reserve(taskNum);
        for (int taskIdx = 0; taskIdx < taskNum; ++taskIdx) {
            const Task& task = taskList[taskIdx];
            if (!CostMatrixGenerator::isTaskValid(amr, task)) continue;
            if (forceCharge[amrIdx] && task.getTaskType() != 3) continue;
            int tId = taskStartIdList[taskIdx]; if (tId < 0) continue;
            // 快速连通性预过滤：AMR起点与任务起点不连通则跳过
            if (!mapInfo.isConnectedById(node_id, tId)) continue;
            auto [tx, ty] = taskStartXY[taskIdx];
            double dx = amrNode.x - tx, dy = amrNode.y - ty;
            // 曼哈顿距离作为默认闵氏距离
            double dist = std::abs(dx) + std::abs(dy);
            const Node& tn = mapInfo.getNodeById(tId);
            double speedNodes = globalSpeed;
            if (amrNode.maxSpeed > 0) speedNodes = std::min(speedNodes, amrNode.maxSpeed);
            if (tn.maxSpeed > 0) speedNodes = std::min(speedNodes, tn.maxSpeed);
            double amrSpeed = amr.getMaxLinearVelocity() > 0 ? amr.getMaxLinearVelocity() : globalSpeed;
            double effSpeed = std::min(speedNodes, amrSpeed);
            if (effSpeed <= 0) effSpeed = globalSpeed;
            double approxTimeMs = (dist / effSpeed) * kMsPerSec;
            heur.emplace_back(taskIdx, approxTimeMs);
        }
        auto rank_cost = [&](const std::pair<int,double>& entry) -> double {
            return apply_task_adjustment(entry.first, entry.second);
        };
        if (topKAgent > 0 && (int)heur.size() > topKAgent) {
            std::nth_element(heur.begin(), heur.begin()+topKAgent, heur.end(),
                             [&](const auto& a, const auto& b){ return rank_cost(a) < rank_cost(b); });
            heur.resize(topKAgent);
        }
        // 对预选集合：逐个异步查询（未完成则回退近似），避免阻塞
        int typ = amr.getDeviceType();
        for (const auto& pr : heur) {
            int taskIdx = pr.first;
            double approxTimeMs = pr.second;
            double spVal = shortestPathUpdater.queryByNodeId(node_id, taskStartIdList[taskIdx], typ);
            double travelMs;
            // 若尚未计算完，回退几何近似；仅在明确不可达时跳过
            if (spVal == static_cast<double>(DistanceState::UNREACHABLE)) continue;
            else if (spVal == static_cast<double>(DistanceState::UNCALCULATED) || !(spVal > 0.0)) travelMs = approxTimeMs;
            else travelMs = spVal * kMsPerSec;
            double offset = 0.0; if (startTimeOffsetMs && (int)startTimeOffsetMs->size()==amrNum) offset = (*startTimeOffsetMs)[amrIdx];
            double val = travelMs + taskSvcMs[taskIdx] + offset;
            val = apply_task_adjustment(taskIdx, val);
            TaskAllocationUtils::sparsePut(taskNum, taskIdx, amrIdx, val);
        }
    }

    // 覆盖纠偏：避免任务被Top-K饿死。若某个任务对所有AMR都为INF，则强制为该任务选择一个最近AMR计算首段
    // 仅在满足可执行约束的AMR内选择，保持语义正确。
    int ensureCover = getenv_int("ALLOC_TOPK_COVERAGE", 1);
    if (ensureCover > 0) {
        for (int t = 0; t < taskNum; ++t) {
            if (taskStartIdList[t] < 0) continue;
            bool covered = false;
            for (int a = 0; a < amrNum; ++a) {
                if (TaskAllocationUtils::getCost(taskNum, t, a) < INF) { covered = true; break; }
            }
            if (covered) continue;
            // 选择最近且可执行的AMR
            int bestAmr = -1; double bestApprox = std::numeric_limits<double>::infinity();
            auto [tx, ty] = taskStartXY[t];
            for (int a = 0; a < amrNum; ++a) {
                const Amr& amr = amrList[a];
                if (!CostMatrixGenerator::isTaskValid(amr, taskList[t])) continue;
                if (forceCharge[a] && taskList[t].getTaskType() != 3) continue;
                int sId = amrStartId[a]; if (sId < 0) continue;
                if (!mapInfo.isConnectedById(sId, taskStartIdList[t])) continue;
                try { const Node& an = mapInfo.getNodeById(sId); (void)an; } catch(...) { continue; }
                const Node& an = mapInfo.getNodeById(sId);
                double dx = an.x - tx, dy = an.y - ty;
                double approx = (std::abs(dx) + std::abs(dy)) / globalSpeed;
                if (approx < bestApprox) { bestApprox = approx; bestAmr = a; }
            }
            if (bestAmr != -1) {
                int typ = amrTypeIndexVec[bestAmr];
                int sId = amrStartId[bestAmr]; if (sId < 0) continue;
                double spMs = shortestPathUpdater.queryByNodeId(sId, taskStartIdList[t], typ);
                if (spMs == static_cast<double>(DistanceState::UNREACHABLE)) continue;
                if (spMs == static_cast<double>(DistanceState::UNCALCULATED) || !(spMs > 0.0)) {
                    // 回退几何近似
                    auto [tx2, ty2] = taskStartXY[t]; const Node& an = mapInfo.getNodeById(sId);
                    double dx = an.x - tx2, dy = an.y - ty2; double dist = std::abs(dx) + std::abs(dy);
                    double speedNodes = globalSpeed; if (an.maxSpeed > 0) speedNodes = std::min(speedNodes, an.maxSpeed);
                    const Node& tn = mapInfo.getNodeById(taskStartIdList[t]); if (tn.maxSpeed > 0) speedNodes = std::min(speedNodes, tn.maxSpeed);
                    double amrSpeed = amrList[bestAmr].getMaxLinearVelocity() > 0 ? amrList[bestAmr].getMaxLinearVelocity() : globalSpeed;
                    double effSpeed = std::min(speedNodes, amrSpeed); if (effSpeed <= 0) effSpeed = globalSpeed;
                    spMs = (dist / effSpeed) * kMsPerSec;
                } else {
                    spMs *= kMsPerSec;
                }
                double offset = 0.0; if (startTimeOffsetMs && (int)startTimeOffsetMs->size()==amrNum) offset = (*startTimeOffsetMs)[bestAmr];
            double val = spMs + taskSvcMs[t] + offset;
            val = apply_task_adjustment(t, val);
            TaskAllocationUtils::sparsePut(taskNum, t, bestAmr, val);
        }
    }
    }

    // 2. 生成 task→task 转移成本，直接写入 costMatrix[i][j][amr]
    for (int amrIdx = 0; amrIdx < amrNum; ++amrIdx) {
        const Amr& amr = amrList[amrIdx];
        int amrTypeIndex = amr.getDeviceType();

        for (int taskIdx1 = 0; taskIdx1 < taskNum; ++taskIdx1) {  // 前序任务（taskList索引）
            const Task& task1 = taskList[taskIdx1];
            if (!CostMatrixGenerator::isTaskValid(amr, task1)) {
                continue;
            }
            // 任务1的时长不直接计入 i->j；方案B 在 i->j 中加 j 的时长
            int task1NodeIdx = -1; // 不再使用索引，保持变量名以减少改动范围
            int task1StartId = taskStartIdList[taskIdx1];
            int task1EndId = taskEndIdList[taskIdx1];
            if (task1StartId < 0 || task1EndId < 0) {
                // 无有效参考节点，跳过
                continue;
            }
            // Top-K 预选：基于几何近似筛选候选，随后用SP求精确时间
            std::vector<std::pair<int,double>> neigh;
            neigh.reserve(taskNum);
            auto [x1, y1] = [&]() -> std::pair<double,double> {
                try { const Node& n = mapInfo.getNodeById(task1EndId); return {n.x, n.y}; }
                catch(...) { return taskStartXY[taskIdx1]; }
            }();
            for (int taskIdx2 = 0; taskIdx2 < taskNum; ++taskIdx2) {
                if (taskIdx1 == taskIdx2) continue;  // 跳过同一任务
                const Task& task2 = taskList[taskIdx2];
                if (!CostMatrixGenerator::isTaskValid(amr, task2)) continue;
                if (forceCharge[amrIdx] && task2.getTaskType() != 3) continue;
                int task2StartId = taskStartIdList[taskIdx2];
                if (task2StartId < 0) continue;
                // 快速连通性预过滤
                if (!mapInfo.isConnectedById(task1EndId, task2StartId)) continue;
                auto [x2,y2] = taskStartXY[taskIdx2];
                double dx = x1 - x2, dy = y1 - y2;
                // 曼哈顿距离作为默认闵氏距离
                double dist = std::abs(dx) + std::abs(dy);
                const Node& n1 = mapInfo.getNodeById(task1EndId);
                const Node& n2 = mapInfo.getNodeById(task2StartId);
                double speedNodes = globalSpeed;
                if (n1.maxSpeed > 0) speedNodes = std::min(speedNodes, n1.maxSpeed);
                if (n2.maxSpeed > 0) speedNodes = std::min(speedNodes, n2.maxSpeed);
                double amrSpeed = amr.getMaxLinearVelocity() > 0 ? amr.getMaxLinearVelocity() : globalSpeed;
                double effSpeed = std::min(speedNodes, amrSpeed);
                if (effSpeed <= 0) effSpeed = globalSpeed;
                double approxTimeMs = (dist / effSpeed) * kMsPerSec;
                neigh.emplace_back(taskIdx2, approxTimeMs);
            }
            auto rank_cost = [&](const std::pair<int,double>& entry) -> double {
                return apply_task_adjustment(entry.first, entry.second);
            };
            if (topKNext > 0 && (int)neigh.size() > topKNext) {
                std::nth_element(neigh.begin(), neigh.begin()+topKNext, neigh.end(),
                                 [&](const auto& a, const auto& b){ return rank_cost(a) < rank_cost(b); });
                neigh.resize(topKNext);
            }
            // 逐个异步查询：task1 end -> Top-K 邻居的起点（未完成则回退近似）
            int typ = amr.getDeviceType();
            for (const auto& pr : neigh) {
                int taskIdx2 = pr.first;
                double spVal = shortestPathUpdater.queryByNodeId(task1EndId, taskStartIdList[taskIdx2], typ);
                double timeMs;
                if (spVal == static_cast<double>(DistanceState::UNREACHABLE)) continue;
                else if (spVal == static_cast<double>(DistanceState::UNCALCULATED) || !(spVal > 0.0)) timeMs = pr.second;
                else timeMs = spVal * kMsPerSec;
                double val = timeMs + taskSvcMs[taskIdx2];
                val = apply_task_adjustment(taskIdx2, val);
                TaskAllocationUtils::sparsePut(taskIdx1, taskIdx2, amrIdx, val);
            }
        }
    }

    // 附加首段时间偏移
    // 稀疏-only：首段时间偏移已在写入时加入，无需二次处理

    return costMatrix;
}

std::vector<std::vector<std::vector<double>>> CostMatrixGenerator::generateCostMatrix(
    const std::vector<Amr>& amrs,
    const std::vector<Task>& taskSet,
    const ShortestPathUpdater& shortestPathUpdater,
    const MapInfo& mapInfo,
    const std::vector<int>& startNodeOverride,
    const std::vector<double>& startTimeOffsetMs
) {
    return generateCostMatrixImpl(amrs, taskSet, shortestPathUpdater, mapInfo, &startNodeOverride, &startTimeOffsetMs);
}

std::vector<std::vector<std::vector<double>>> CostMatrixGenerator::generateCostMatrix(
    const std::vector<Amr>& amrs,
    const std::vector<Task>& taskSet,
    const ShortestPathUpdater& shortestPathUpdater,
    const MapInfo& mapInfo
) {
    return generateCostMatrixImpl(amrs, taskSet, shortestPathUpdater, mapInfo, nullptr, nullptr);
}


// 检查任务是否有效（类型匹配）
bool CostMatrixGenerator::isTaskValid(Amr amr, Task task) {
    // 优先级阈值
    if (amr.getPriorityThreshold() > task.getPriority()) return false;
    // 不可用状态直接过滤
    if (amr.isUnavailable()) return false;
    // 最低电量约束（0表示不限制）
    if (task.getMinBatteryLevel() > 0 && amr.getBatteryLevel() < task.getMinBatteryLevel()) return false;
    // 设备ID白名单（为空则不限制）
    const auto& ids = task.getDeviceIds();
    if (!ids.empty()) {
        bool ok = false; for (const auto& id : ids) if (id == amr.getDeviceId()) { ok = true; break; }
        if (!ok) return false;
    }
    // 新约束：非紧急任务仅允许空闲AMR接取
    bool isEmergencyTask = (task.getPriority() >= TaskPolicy::EMERGENCY_LEVEL);
    if (!isEmergencyTask && !amr.isIdle()) {
        return false;
    }
    return true;
}
