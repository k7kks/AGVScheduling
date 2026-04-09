#pragma once

#include <vector>

#include "data/MapInfo.h"

namespace CongestionRegionUtils {

int autoRegionGrid(int itemCount, int minGrid, int maxGrid);

bool isPassableRegionNode(const Node& node);

void gatherNeighborIndices(const MapInfo& mapInfo, int idx, std::vector<int>& out);

int buildGraphRegions(const MapInfo& mapInfo,
                      int targetRegions,
                      std::vector<int>& regionByIndex,
                      std::vector<int>& seedIndices,
                      std::vector<char>* bridgeRegionMask,
                      bool enableBridgeRegions);

}  // namespace CongestionRegionUtils
