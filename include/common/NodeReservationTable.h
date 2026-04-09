#ifndef NODE_RESERVATION_TABLE_H
#define NODE_RESERVATION_TABLE_H

#include <array>
#include <chrono>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

class MapInfo;

// Shared, concurrent node reservation/occupancy table.
// - Key: nodeId
// - Value: who holds it + why + optional TTL (failsafe cleanup)
class NodeReservationTable {
public:
    enum class HoldReason : int {
        RESERVED_PATH = 0,   // Reserved as part of a planned path window
        WAITING_POINT = 1,   // AGV is waiting on this node
        TEMP_GOAL = 2,       // Temporary goal for deadlock relief
        OTHER = 99
    };

    struct HoldEntry {
        int nodeId = -1;
        std::string ownerAgvId;
        HoldReason reason = HoldReason::OTHER;
        std::string detail;
        std::chrono::steady_clock::time_point updatedAt{};
        std::chrono::steady_clock::time_point expiresAt{};  // expiresAt==time_point{} => never expires
    };

    NodeReservationTable() = default;

    // Configure distance-based conflict groups from current map.
    // Nodes whose Euclidean distance is <= thresholdMm will be treated as mutually exclusive.
    // Exact-node conflict is always enforced even when thresholdMm <= 0.
    void configureDistanceConflict(const MapInfo& mapInfo, double thresholdMm);

    // Try to reserve a node. If the node is already held by another owner and not expired, returns false.
    // If held by the same owner, refreshes metadata and returns true.
    bool tryReserve(int nodeId,
                    const std::string& ownerAgvId,
                    HoldReason reason,
                    const std::string& detail = {},
                    std::chrono::milliseconds ttl = std::chrono::milliseconds(0));

    // Release a node held by owner (no-op if not held or held by others).
    void release(int nodeId, const std::string& ownerAgvId);

    // Release all nodes held by owner.
    void releaseAllByOwner(const std::string& ownerAgvId);

    // Release all nodes held by owner except keepNodeId (best-effort).
    void releaseAllByOwnerExcept(const std::string& ownerAgvId, int keepNodeId);

    // Release all nodes held by owner except keepNodeIds (best-effort).
    void releaseAllByOwnerExceptSet(const std::string& ownerAgvId, const std::vector<int>& keepNodeIds);

    // Snapshot held nodeIds (excluding the owner's own holds when excludeOwnerAgvId is provided).
    std::vector<int> snapshotHeldNodes(const std::string& excludeOwnerAgvId = {}) const;

    // Snapshot blocked nodeIds after expanding held nodes by the configured distance-conflict rule.
    std::vector<int> snapshotBlockedNodes(const std::string& excludeOwnerAgvId = {}) const;

    // Get hold entry for a node (nullopt if not held / expired).
    std::optional<HoldEntry> get(int nodeId) const;

    // Find a hold that blocks reserving/using nodeId, including nearby conflicting nodes.
    std::optional<HoldEntry> findBlockingHold(int nodeId,
                                             const std::string& excludeOwnerAgvId = {}) const;

    // Best-effort cleanup of expired entries.
    void cleanupExpired();

private:
    struct EntryInternal {
        std::string ownerAgvId;
        HoldReason reason = HoldReason::OTHER;
        std::string detail;
        std::chrono::steady_clock::time_point updatedAt{};
        std::chrono::steady_clock::time_point expiresAt{};
    };

    struct Shard {
        mutable std::shared_mutex mutex;
        std::unordered_map<int, EntryInternal> entries;
    };

    struct DistanceConflictIndex {
        double thresholdMm = 0.0;
        std::unordered_map<int, std::vector<int>> nodeIdsByNodeId;
    };

    static constexpr size_t kShardCount = 64;
    std::array<Shard, kShardCount> shards_{};
    mutable std::shared_mutex distanceConflictMutex_;
    DistanceConflictIndex distanceConflictIndex_{};

    static size_t shardOf(int nodeId) {
        return std::hash<int>{}(nodeId) % kShardCount;
    }

    static bool isExpired(const EntryInternal& e, std::chrono::steady_clock::time_point now) {
        if (e.expiresAt == std::chrono::steady_clock::time_point{}) return false;
        return now >= e.expiresAt;
    }

    std::vector<int> conflictNodeIdsFor(int nodeId) const;
};

#endif  // NODE_RESERVATION_TABLE_H
