#include "common/NodeReservationTable.h"

#include "data/MapInfo.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <mutex>

void NodeReservationTable::configureDistanceConflict(const MapInfo& mapInfo, double thresholdMm) {
    DistanceConflictIndex nextIndex;
    if (std::isfinite(thresholdMm) && thresholdMm > 0.0) {
        nextIndex.thresholdMm = thresholdMm;
    }

    const auto& nodes = mapInfo.getNodes();
    nextIndex.nodeIdsByNodeId.reserve(nodes.size());
    if (nodes.empty()) {
        std::unique_lock<std::shared_mutex> lk(distanceConflictMutex_);
        distanceConflictIndex_ = std::move(nextIndex);
        return;
    }

    if (!(nextIndex.thresholdMm > 0.0)) {
        for (const auto& node : nodes) {
            if (node.id < 0) continue;
            nextIndex.nodeIdsByNodeId[node.id] = {node.id};
        }
        std::unique_lock<std::shared_mutex> lk(distanceConflictMutex_);
        distanceConflictIndex_ = std::move(nextIndex);
        return;
    }

    const double cellSize = nextIndex.thresholdMm;
    const double thresholdSq = nextIndex.thresholdMm * nextIndex.thresholdMm;
    auto cell_coord = [&](double value) -> long long {
        return static_cast<long long>(std::floor(value / cellSize));
    };
    auto cell_key = [](long long cx, long long cy) -> long long {
        const auto hi = static_cast<std::uint64_t>(static_cast<std::int64_t>(cx));
        const auto lo = static_cast<std::uint64_t>(static_cast<std::int64_t>(cy));
        return static_cast<long long>((hi << 32) ^ (lo & 0xffffffffULL));
    };

    std::unordered_map<long long, std::vector<int>> nodeIndicesByCell;
    nodeIndicesByCell.reserve(nodes.size());
    for (int idx = 0; idx < static_cast<int>(nodes.size()); ++idx) {
        const auto& node = nodes[static_cast<size_t>(idx)];
        if (node.id < 0) continue;
        nodeIndicesByCell[cell_key(cell_coord(node.x), cell_coord(node.y))].push_back(idx);
    }

    for (int idx = 0; idx < static_cast<int>(nodes.size()); ++idx) {
        const auto& node = nodes[static_cast<size_t>(idx)];
        if (node.id < 0) continue;

        std::vector<int> nearbyNodeIds;
        nearbyNodeIds.push_back(node.id);
        const long long cellX = cell_coord(node.x);
        const long long cellY = cell_coord(node.y);
        for (long long dx = -1; dx <= 1; ++dx) {
            for (long long dy = -1; dy <= 1; ++dy) {
                auto it = nodeIndicesByCell.find(cell_key(cellX + dx, cellY + dy));
                if (it == nodeIndicesByCell.end()) continue;
                for (int otherIdx : it->second) {
                    if (otherIdx == idx) continue;
                    const auto& other = nodes[static_cast<size_t>(otherIdx)];
                    if (other.id < 0) continue;
                    const double diffX = node.x - other.x;
                    const double diffY = node.y - other.y;
                    const double distSq = diffX * diffX + diffY * diffY;
                    if (distSq <= thresholdSq) {
                        nearbyNodeIds.push_back(other.id);
                    }
                }
            }
        }
        std::sort(nearbyNodeIds.begin(), nearbyNodeIds.end());
        nearbyNodeIds.erase(std::unique(nearbyNodeIds.begin(), nearbyNodeIds.end()), nearbyNodeIds.end());
        nextIndex.nodeIdsByNodeId[node.id] = std::move(nearbyNodeIds);
    }

    std::unique_lock<std::shared_mutex> lk(distanceConflictMutex_);
    distanceConflictIndex_ = std::move(nextIndex);
}

std::vector<int> NodeReservationTable::conflictNodeIdsFor(int nodeId) const {
    std::shared_lock<std::shared_mutex> lk(distanceConflictMutex_);
    auto it = distanceConflictIndex_.nodeIdsByNodeId.find(nodeId);
    if (it == distanceConflictIndex_.nodeIdsByNodeId.end() || it->second.empty()) {
        return {nodeId};
    }
    std::vector<int> out;
    out.reserve(it->second.size());
    out.push_back(nodeId);
    for (int otherNodeId : it->second) {
        if (otherNodeId == nodeId) continue;
        out.push_back(otherNodeId);
    }
    return out;
}

bool NodeReservationTable::tryReserve(int nodeId,
                                      const std::string& ownerAgvId,
                                      HoldReason reason,
                                      const std::string& detail,
                                      std::chrono::milliseconds ttl) {
    if (nodeId < 0 || ownerAgvId.empty()) return false;
    const auto now = std::chrono::steady_clock::now();
    const auto expiresAt = (ttl.count() > 0) ? (now + ttl) : std::chrono::steady_clock::time_point{};

    std::vector<int> conflictNodeIds = conflictNodeIdsFor(nodeId);
    std::vector<size_t> shardIds;
    shardIds.reserve(conflictNodeIds.size());
    for (int conflictNodeId : conflictNodeIds) {
        shardIds.push_back(shardOf(conflictNodeId));
    }
    std::sort(shardIds.begin(), shardIds.end());
    shardIds.erase(std::unique(shardIds.begin(), shardIds.end()), shardIds.end());

    std::vector<std::unique_lock<std::shared_mutex>> locks;
    locks.reserve(shardIds.size());
    for (size_t shardId : shardIds) {
        locks.emplace_back(shards_[shardId].mutex);
    }

    for (int conflictNodeId : conflictNodeIds) {
        auto& entries = shards_[shardOf(conflictNodeId)].entries;
        auto it = entries.find(conflictNodeId);
        if (it != entries.end() && isExpired(it->second, now)) {
            entries.erase(it);
        }
    }

    for (int conflictNodeId : conflictNodeIds) {
        auto& entries = shards_[shardOf(conflictNodeId)].entries;
        auto it = entries.find(conflictNodeId);
        if (it == entries.end()) continue;
        if (it->second.ownerAgvId != ownerAgvId) {
            return false;
        }
    }

    auto& targetEntries = shards_[shardOf(nodeId)].entries;
    auto it = targetEntries.find(nodeId);
    if (it == targetEntries.end()) {
        EntryInternal e;
        e.ownerAgvId = ownerAgvId;
        e.reason = reason;
        e.detail = detail;
        e.updatedAt = now;
        e.expiresAt = expiresAt;
        targetEntries.emplace(nodeId, std::move(e));
        return true;
    }

    it->second.reason = reason;
    it->second.detail = detail;
    it->second.updatedAt = now;
    it->second.expiresAt = expiresAt;
    return true;
}

void NodeReservationTable::release(int nodeId, const std::string& ownerAgvId) {
    if (nodeId < 0 || ownerAgvId.empty()) return;
    const auto now = std::chrono::steady_clock::now();
    Shard& shard = shards_[shardOf(nodeId)];
    std::unique_lock<std::shared_mutex> lk(shard.mutex);
    auto it = shard.entries.find(nodeId);
    if (it == shard.entries.end()) return;
    if (isExpired(it->second, now)) {
        shard.entries.erase(it);
        return;
    }
    if (it->second.ownerAgvId != ownerAgvId) return;
    shard.entries.erase(it);
}

void NodeReservationTable::releaseAllByOwner(const std::string& ownerAgvId) {
    if (ownerAgvId.empty()) return;
    const auto now = std::chrono::steady_clock::now();
    for (auto& shard : shards_) {
        std::unique_lock<std::shared_mutex> lk(shard.mutex);
        for (auto it = shard.entries.begin(); it != shard.entries.end();) {
            if (isExpired(it->second, now) || it->second.ownerAgvId == ownerAgvId) {
                it = shard.entries.erase(it);
            } else {
                ++it;
            }
        }
    }
}

void NodeReservationTable::releaseAllByOwnerExcept(const std::string& ownerAgvId, int keepNodeId) {
    if (ownerAgvId.empty()) return;
    const auto now = std::chrono::steady_clock::now();
    for (auto& shard : shards_) {
        std::unique_lock<std::shared_mutex> lk(shard.mutex);
        for (auto it = shard.entries.begin(); it != shard.entries.end();) {
            const int nodeId = it->first;
            const EntryInternal& e = it->second;
            if (isExpired(e, now)) {
                it = shard.entries.erase(it);
                continue;
            }
            if (e.ownerAgvId == ownerAgvId && nodeId != keepNodeId) {
                it = shard.entries.erase(it);
                continue;
            }
            ++it;
        }
    }
}

void NodeReservationTable::releaseAllByOwnerExceptSet(const std::string& ownerAgvId, const std::vector<int>& keepNodeIds) {
    if (ownerAgvId.empty()) return;
    const auto now = std::chrono::steady_clock::now();
    auto keep = [&](int nodeId) -> bool {
        for (int k : keepNodeIds) {
            if (k == nodeId) return true;
        }
        return false;
    };
    for (auto& shard : shards_) {
        std::unique_lock<std::shared_mutex> lk(shard.mutex);
        for (auto it = shard.entries.begin(); it != shard.entries.end();) {
            const int nodeId = it->first;
            const EntryInternal& e = it->second;
            if (isExpired(e, now)) {
                it = shard.entries.erase(it);
                continue;
            }
            if (e.ownerAgvId == ownerAgvId && !keep(nodeId)) {
                it = shard.entries.erase(it);
                continue;
            }
            ++it;
        }
    }
}

std::vector<int> NodeReservationTable::snapshotHeldNodes(const std::string& excludeOwnerAgvId) const {
    const auto now = std::chrono::steady_clock::now();
    std::vector<int> out;
    for (const auto& shard : shards_) {
        std::shared_lock<std::shared_mutex> lk(shard.mutex);
        for (const auto& kv : shard.entries) {
            const int nodeId = kv.first;
            const EntryInternal& e = kv.second;
            if (isExpired(e, now)) continue;
            if (!excludeOwnerAgvId.empty() && e.ownerAgvId == excludeOwnerAgvId) continue;
            out.push_back(nodeId);
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::vector<int> NodeReservationTable::snapshotBlockedNodes(const std::string& excludeOwnerAgvId) const {
    std::vector<int> heldNodeIds = snapshotHeldNodes(excludeOwnerAgvId);
    if (heldNodeIds.empty()) return heldNodeIds;

    std::vector<int> out;
    {
        std::shared_lock<std::shared_mutex> lk(distanceConflictMutex_);
        for (int heldNodeId : heldNodeIds) {
            auto it = distanceConflictIndex_.nodeIdsByNodeId.find(heldNodeId);
            if (it == distanceConflictIndex_.nodeIdsByNodeId.end() || it->second.empty()) {
                out.push_back(heldNodeId);
                continue;
            }
            out.insert(out.end(), it->second.begin(), it->second.end());
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::optional<NodeReservationTable::HoldEntry> NodeReservationTable::get(int nodeId) const {
    if (nodeId < 0) return std::nullopt;
    const auto now = std::chrono::steady_clock::now();
    const auto& shard = shards_[shardOf(nodeId)];
    std::shared_lock<std::shared_mutex> lk(shard.mutex);
    auto it = shard.entries.find(nodeId);
    if (it == shard.entries.end()) return std::nullopt;
    if (isExpired(it->second, now)) return std::nullopt;
    HoldEntry out;
    out.nodeId = nodeId;
    out.ownerAgvId = it->second.ownerAgvId;
    out.reason = it->second.reason;
    out.detail = it->second.detail;
    out.updatedAt = it->second.updatedAt;
    out.expiresAt = it->second.expiresAt;
    return out;
}

std::optional<NodeReservationTable::HoldEntry> NodeReservationTable::findBlockingHold(
    int nodeId,
    const std::string& excludeOwnerAgvId) const {
    if (nodeId < 0) return std::nullopt;
    const auto now = std::chrono::steady_clock::now();
    std::vector<int> conflictNodeIds = conflictNodeIdsFor(nodeId);
    std::vector<size_t> shardIds;
    shardIds.reserve(conflictNodeIds.size());
    for (int conflictNodeId : conflictNodeIds) {
        shardIds.push_back(shardOf(conflictNodeId));
    }
    std::sort(shardIds.begin(), shardIds.end());
    shardIds.erase(std::unique(shardIds.begin(), shardIds.end()), shardIds.end());

    std::vector<std::shared_lock<std::shared_mutex>> locks;
    locks.reserve(shardIds.size());
    for (size_t shardId : shardIds) {
        locks.emplace_back(shards_[shardId].mutex);
    }

    for (int conflictNodeId : conflictNodeIds) {
        const auto& entries = shards_[shardOf(conflictNodeId)].entries;
        auto it = entries.find(conflictNodeId);
        if (it == entries.end() || isExpired(it->second, now)) continue;
        if (!excludeOwnerAgvId.empty() && it->second.ownerAgvId == excludeOwnerAgvId) continue;
        HoldEntry out;
        out.nodeId = conflictNodeId;
        out.ownerAgvId = it->second.ownerAgvId;
        out.reason = it->second.reason;
        out.detail = it->second.detail;
        out.updatedAt = it->second.updatedAt;
        out.expiresAt = it->second.expiresAt;
        return out;
    }
    return std::nullopt;
}

void NodeReservationTable::cleanupExpired() {
    const auto now = std::chrono::steady_clock::now();
    for (auto& shard : shards_) {
        std::unique_lock<std::shared_mutex> lk(shard.mutex);
        for (auto it = shard.entries.begin(); it != shard.entries.end();) {
            if (isExpired(it->second, now)) {
                it = shard.entries.erase(it);
            } else {
                ++it;
            }
        }
    }
}
