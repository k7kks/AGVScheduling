#include "common/AmrPositionResolver.h"

namespace AmrPositionResolver {

int resolveStartNodeId(const Amr& amr, const MapInfo& mapInfo) {
    // 1) 优先使用 currentNodeId（如果提供）
    if (amr.getCurrentNodeId() != -1) return amr.getCurrentNodeId();

    int x = amr.getX();
    int y = amr.getY();
    const Point& nextPt = amr.getNextDestinationPoint();
    int nxt = nextPt.getNodeId();

    // 2) 使用 nextDestinationPointId 为中心，在其自身+邻居中按 (x,y) 匹配最近
    if (nxt != -1) {
        try {
            return mapInfo.findNearestNodeIdFromNeighbors(x, y, nxt);
        } catch (...) {
            // ref 节点无效则回退到全图
        }
    } else {
        // 2b) 若 nextPoint 没有 nodeId，但提供了坐标，则先按 nextPoint 坐标就近匹配
        int nx = nextPt.getX();
        int ny = nextPt.getY();
        if (!(nx == 0 && ny == 0)) {
            try {
                return mapInfo.findNearestNodeId(nx, ny);
            } catch (...) {
                // fall through
            }
        }
    }

    // 3) 兜底：全图就近
    return mapInfo.findNearestNodeId(x, y);
}

} // namespace AmrPositionResolver
