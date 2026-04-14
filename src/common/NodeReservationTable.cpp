#include "common/NodeReservationTable.h"

#include "data/MapInfo.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <mutex>

namespace {

static bool keep_group_key(const std::vector<std::string>& keys, const std::string& groupKey) {
    return std::find(keys.begin(), keys.end(), groupKey) != keys.end();
}

}  // namespace

void NodeReservationTable::erasePreReservationNodeLocked(int nodeId, const std::string& ownerAgvId) {
    if (nodeId < 0 || ownerAgvId.empty()) return;
    auto it = preReservationEntries_.find(nodeId);
    if (it != preReservationEntries_.end() && it->second.ownerAgvId == ownerAgvId) {
        preReservationEntries_.erase(it);
    }
    auto ownerIt = preReservationNodesByOwner_.find(ownerAgvId);
    if (ownerIt == preReservationNodesByOwner_.end()) return;
    auto& nodes = ownerIt->second;
    nodes.erase(std::remove(nodes.begin(), nodes.end(), nodeId), nodes.end());
    if (nodes.empty()) {
        preReservationNodesByOwner_.erase(ownerIt);
    }
}

void NodeReservationTable::clearPreReservationsByOwnerLocked(const std::string& ownerAgvId) {
    if (ownerAgvId.empty()) return;
    auto ownerIt = preReservationNodesByOwner_.find(ownerAgvId);
    if (ownerIt == preReservationNodesByOwner_.end()) return;
    const std::vector<int> nodes = ownerIt->second;
    preReservationNodesByOwner_.erase(ownerIt);
    for (int nodeId : nodes) {
        auto it = preReservationEntries_.find(nodeId);
        if (it != preReservationEntries_.end() && it->second.ownerAgvId == ownerAgvId) {
            preReservationEntries_.erase(it);
        }
    }
}

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

    std::unique_lock<std::shared_mutex> preLk(preReservationMutex_);
    auto preIt = preReservationEntries_.find(nodeId);
    if (preIt != preReservationEntries_.end() && preIt->second.ownerAgvId != ownerAgvId) {
        return false;
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
        erasePreReservationNodeLocked(nodeId, ownerAgvId);
        return true;
    }

    it->second.reason = reason;
    it->second.detail = detail;
    it->second.updatedAt = now;
    it->second.expiresAt = expiresAt;
    erasePreReservationNodeLocked(nodeId, ownerAgvId);
    return true;
}

void NodeReservationTable::updatePreReservations(const std::string& ownerAgvId,
                                                 const std::vector<int>& nodeIds) {
    if (ownerAgvId.empty()) return;
    std::vector<int> desired = nodeIds;
    desired.erase(std::remove_if(desired.begin(), desired.end(), [](int nodeId) {
        return nodeId < 0;
    }), desired.end());
    std::sort(desired.begin(), desired.end());
    desired.erase(std::unique(desired.begin(), desired.end()), desired.end());

    std::unique_lock<std::shared_mutex> lk(preReservationMutex_);
    std::vector<int> old;
    auto oldIt = preReservationNodesByOwner_.find(ownerAgvId);
    if (oldIt != preReservationNodesByOwner_.end()) {
        old = oldIt->second;
    }
    if (old == desired) {
        const auto now = std::chrono::steady_clock::now();
        for (int nodeId : desired) {
            auto it = preReservationEntries_.find(nodeId);
            if (it != preReservationEntries_.end() && it->second.ownerAgvId == ownerAgvId) {
                it->second.updatedAt = now;
            }
        }
        return;
    }

    clearPreReservationsByOwnerLocked(ownerAgvId);
    if (desired.empty()) return;

    const auto now = std::chrono::steady_clock::now();
    std::vector<int> accepted;
    accepted.reserve(desired.size());
    for (int nodeId : desired) {
        auto [it, inserted] = preReservationEntries_.try_emplace(nodeId);
        if (!inserted && it->second.ownerAgvId != ownerAgvId) {
            continue;
        }
        it->second.ownerAgvId = ownerAgvId;
        it->second.updatedAt = now;
        accepted.push_back(nodeId);
    }
    if (!accepted.empty()) {
        preReservationNodesByOwner_[ownerAgvId] = std::move(accepted);
    }
}

void NodeReservationTable::clearPreReservationsByOwner(const std::string& ownerAgvId) {
    if (ownerAgvId.empty()) return;
    std::unique_lock<std::shared_mutex> lk(preReservationMutex_);
    clearPreReservationsByOwnerLocked(ownerAgvId);
}

bool NodeReservationTable::ownerHasPreReservations(const std::string& ownerAgvId) const {
    if (ownerAgvId.empty()) return false;
    std::shared_lock<std::shared_mutex> lk(preReservationMutex_);
    auto it = preReservationNodesByOwner_.find(ownerAgvId);
    return it != preReservationNodesByOwner_.end() && !it->second.empty();
}

std::optional<NodeReservationTable::PreReservationEntry>
NodeReservationTable::getPreReservation(int nodeId) const {
    if (nodeId < 0) return std::nullopt;
    std::shared_lock<std::shared_mutex> lk(preReservationMutex_);
    auto it = preReservationEntries_.find(nodeId);
    if (it == preReservationEntries_.end() || it->second.ownerAgvId.empty()) return std::nullopt;
    PreReservationEntry entry;
    entry.nodeId = nodeId;
    entry.ownerAgvId = it->second.ownerAgvId;
    entry.updatedAt = it->second.updatedAt;
    return entry;
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
    clearPreReservationsByOwner(ownerAgvId);
    releaseAllDirectionalGroupsByOwner(ownerAgvId);
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
    std::shared_lock<std::shared_mutex> preLk(preReservationMutex_);
    auto preIt = preReservationEntries_.find(nodeId);
    if (preIt != preReservationEntries_.end() &&
        !preIt->second.ownerAgvId.empty() &&
        (excludeOwnerAgvId.empty() || preIt->second.ownerAgvId != excludeOwnerAgvId)) {
        HoldEntry out;
        out.nodeId = nodeId;
        out.ownerAgvId = preIt->second.ownerAgvId;
        out.reason = HoldReason::PRE_RESERVATION;
        out.detail = "pre_reserved";
        out.updatedAt = preIt->second.updatedAt;
        out.expiresAt = std::chrono::steady_clock::time_point{};
        return out;
    }
    return std::nullopt;
}

bool NodeReservationTable::tryReserveDirectionalGroup(const std::string& groupKey,
                                                      const std::string& ownerAgvId,
                                                      int direction,
                                                      int sameDirectionCapacity,
                                                      const std::string& detail,
                                                      std::chrono::milliseconds ttl) {
    if (groupKey.empty() || ownerAgvId.empty()) return false;
    const auto now = std::chrono::steady_clock::now();
    const auto expiresAt = (ttl.count() > 0) ? (now + ttl) : std::chrono::steady_clock::time_point{};
    const int normalizedDir = (direction > 0) ? 1 : ((direction < 0) ? -1 : 0);
    const int normalizedCap = std::max(1, sameDirectionCapacity);

    std::unique_lock<std::shared_mutex> lk(directionalGroupMutex_);
    auto& groupEntries = directionalGroupEntries_[groupKey];
    for (auto it = groupEntries.begin(); it != groupEntries.end();) {
        if (isExpired(it->second, now)) {
            it = groupEntries.erase(it);
        } else {
            ++it;
        }
    }

    int sameDirOwners = 0;
    bool blocked = false;
    for (const auto& kv : groupEntries) {
        if (kv.first == ownerAgvId) continue;
        const auto& e = kv.second;
        const int otherDir = (e.direction > 0) ? 1 : ((e.direction < 0) ? -1 : 0);
        if (normalizedDir == 0 || otherDir == 0) {
            blocked = true;
            break;
        }
        if (otherDir != normalizedDir) {
            blocked = true;
            break;
        }
        sameDirOwners += 1;
    }
    if (!blocked && normalizedDir != 0 && sameDirOwners >= normalizedCap) {
        blocked = true;
    }
    if (blocked) return false;

    auto& entry = groupEntries[ownerAgvId];
    entry.direction = normalizedDir;
    entry.sameDirectionCapacity = normalizedCap;
    entry.detail = detail;
    entry.updatedAt = now;
    entry.expiresAt = expiresAt;
    return true;
}

void NodeReservationTable::releaseDirectionalGroup(const std::string& groupKey,
                                                   const std::string& ownerAgvId) {
    if (groupKey.empty() || ownerAgvId.empty()) return;
    const auto now = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(directionalGroupMutex_);
    auto it = directionalGroupEntries_.find(groupKey);
    if (it == directionalGroupEntries_.end()) return;
    auto& groupEntries = it->second;
    for (auto jt = groupEntries.begin(); jt != groupEntries.end();) {
        if (isExpired(jt->second, now) || jt->first == ownerAgvId) {
            jt = groupEntries.erase(jt);
        } else {
            ++jt;
        }
    }
    if (groupEntries.empty()) directionalGroupEntries_.erase(it);
}

void NodeReservationTable::releaseAllDirectionalGroupsByOwner(const std::string& ownerAgvId) {
    if (ownerAgvId.empty()) return;
    const auto now = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(directionalGroupMutex_);
    for (auto it = directionalGroupEntries_.begin(); it != directionalGroupEntries_.end();) {
        auto& groupEntries = it->second;
        for (auto jt = groupEntries.begin(); jt != groupEntries.end();) {
            if (isExpired(jt->second, now) || jt->first == ownerAgvId) {
                jt = groupEntries.erase(jt);
            } else {
                ++jt;
            }
        }
        if (groupEntries.empty()) {
            it = directionalGroupEntries_.erase(it);
        } else {
            ++it;
        }
    }
}

void NodeReservationTable::releaseAllDirectionalGroupsByOwnerExceptSet(
    const std::string& ownerAgvId,
    const std::vector<std::string>& keepGroupKeys) {
    if (ownerAgvId.empty()) return;
    const auto now = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(directionalGroupMutex_);
    for (auto it = directionalGroupEntries_.begin(); it != directionalGroupEntries_.end();) {
        auto& groupEntries = it->second;
        for (auto jt = groupEntries.begin(); jt != groupEntries.end();) {
            if (isExpired(jt->second, now)) {
                jt = groupEntries.erase(jt);
                continue;
            }
            if (jt->first == ownerAgvId && !keep_group_key(keepGroupKeys, it->first)) {
                jt = groupEntries.erase(jt);
                continue;
            }
            ++jt;
        }
        if (groupEntries.empty()) {
            it = directionalGroupEntries_.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<NodeReservationTable::DirectionalGroupHoldEntry>
NodeReservationTable::snapshotDirectionalGroupHolds(const std::string& groupKey,
                                                    const std::string& excludeOwnerAgvId) const {
    std::vector<DirectionalGroupHoldEntry> out;
    if (groupKey.empty()) return out;
    const auto now = std::chrono::steady_clock::now();
    std::shared_lock<std::shared_mutex> lk(directionalGroupMutex_);
    auto it = directionalGroupEntries_.find(groupKey);
    if (it == directionalGroupEntries_.end()) return out;
    out.reserve(it->second.size());
    for (const auto& kv : it->second) {
        if (!excludeOwnerAgvId.empty() && kv.first == excludeOwnerAgvId) continue;
        if (isExpired(kv.second, now)) continue;
        DirectionalGroupHoldEntry entry;
        entry.groupKey = groupKey;
        entry.ownerAgvId = kv.first;
        entry.direction = kv.second.direction;
        entry.sameDirectionCapacity = kv.second.sameDirectionCapacity;
        entry.detail = kv.second.detail;
        entry.updatedAt = kv.second.updatedAt;
        entry.expiresAt = kv.second.expiresAt;
        out.push_back(std::move(entry));
    }
    return out;
}

std::optional<NodeReservationTable::DirectionalGroupHoldEntry>
NodeReservationTable::getDirectionalGroupHold(const std::string& groupKey,
                                              const std::string& ownerAgvId) const {
    if (groupKey.empty() || ownerAgvId.empty()) return std::nullopt;
    const auto now = std::chrono::steady_clock::now();
    std::shared_lock<std::shared_mutex> lk(directionalGroupMutex_);
    auto it = directionalGroupEntries_.find(groupKey);
    if (it == directionalGroupEntries_.end()) return std::nullopt;
    auto jt = it->second.find(ownerAgvId);
    if (jt == it->second.end() || isExpired(jt->second, now)) return std::nullopt;
    DirectionalGroupHoldEntry entry;
    entry.groupKey = groupKey;
    entry.ownerAgvId = ownerAgvId;
    entry.direction = jt->second.direction;
    entry.sameDirectionCapacity = jt->second.sameDirectionCapacity;
    entry.detail = jt->second.detail;
    entry.updatedAt = jt->second.updatedAt;
    entry.expiresAt = jt->second.expiresAt;
    return entry;
}

std::optional<NodeReservationTable::DirectionalGroupHoldEntry>
NodeReservationTable::findBlockingDirectionalGroup(const std::string& groupKey,
                                                   const std::string& ownerAgvId,
                                                   int direction,
                                                   int sameDirectionCapacity) const {
    if (groupKey.empty() || ownerAgvId.empty()) return std::nullopt;
    const auto now = std::chrono::steady_clock::now();
    const int normalizedDir = (direction > 0) ? 1 : ((direction < 0) ? -1 : 0);
    const int normalizedCap = std::max(1, sameDirectionCapacity);
    std::shared_lock<std::shared_mutex> lk(directionalGroupMutex_);
    auto it = directionalGroupEntries_.find(groupKey);
    if (it == directionalGroupEntries_.end()) return std::nullopt;

    int sameDirOwners = 0;
    std::optional<DirectionalGroupHoldEntry> fallback;
    for (const auto& kv : it->second) {
        if (kv.first == ownerAgvId || isExpired(kv.second, now)) continue;
        const int otherDir = (kv.second.direction > 0) ? 1 : ((kv.second.direction < 0) ? -1 : 0);
        DirectionalGroupHoldEntry entry;
        entry.groupKey = groupKey;
        entry.ownerAgvId = kv.first;
        entry.direction = kv.second.direction;
        entry.sameDirectionCapacity = kv.second.sameDirectionCapacity;
        entry.detail = kv.second.detail;
        entry.updatedAt = kv.second.updatedAt;
        entry.expiresAt = kv.second.expiresAt;
        if (normalizedDir == 0 || otherDir == 0 || otherDir != normalizedDir) {
            return entry;
        }
        sameDirOwners += 1;
        fallback = entry;
    }
    if (normalizedDir != 0 && sameDirOwners >= normalizedCap) {
        return fallback;
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
    {
        std::unique_lock<std::shared_mutex> lk(preReservationMutex_);
        for (auto it = preReservationEntries_.begin(); it != preReservationEntries_.end();) {
            if (it->second.ownerAgvId.empty()) {
                it = preReservationEntries_.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = preReservationNodesByOwner_.begin(); it != preReservationNodesByOwner_.end();) {
            auto& nodes = it->second;
            nodes.erase(std::remove_if(nodes.begin(), nodes.end(), [&](int nodeId) {
                auto pit = preReservationEntries_.find(nodeId);
                return pit == preReservationEntries_.end() || pit->second.ownerAgvId != it->first;
            }), nodes.end());
            if (nodes.empty()) {
                it = preReservationNodesByOwner_.erase(it);
            } else {
                ++it;
            }
        }
    }
    std::unique_lock<std::shared_mutex> lk(directionalGroupMutex_);
    for (auto it = directionalGroupEntries_.begin(); it != directionalGroupEntries_.end();) {
        auto& groupEntries = it->second;
        for (auto jt = groupEntries.begin(); jt != groupEntries.end();) {
            if (isExpired(jt->second, now)) {
                jt = groupEntries.erase(jt);
            } else {
                ++jt;
            }
        }
        if (groupEntries.empty()) {
            it = directionalGroupEntries_.erase(it);
        } else {
            ++it;
        }
    }
}
