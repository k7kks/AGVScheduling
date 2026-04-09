#ifndef AMR_POSITION_RESOLVER_H
#define AMR_POSITION_RESOLVER_H

#include "data/Amr.h"
#include "data/MapInfo.h"

namespace AmrPositionResolver {

// 统一的AMR起点节点解析逻辑：
// 1) 若提供 currentNodeId 则直接返回
// 2) 否则若提供 nextDestinationPointId，则以其为中心，在“自身+前驱+后继”中选择与当前(x,y)最近的节点
// 3) 实在没有，则在全图中按(x,y)查最近节点
int resolveStartNodeId(const Amr& amr, const MapInfo& mapInfo);

}

#endif // AMR_POSITION_RESOLVER_H

