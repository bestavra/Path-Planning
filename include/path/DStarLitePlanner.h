#pragma once

#include "map/Map.h"
#include "path/IPathPlanner.h"
#include "path/PlannerTypes.h"

#include <functional>
#include <glm/glm.hpp>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace planner {

/**
 * @brief Incremental D* Lite planner capable of responding to dynamic cost changes, planning on 8-connected grids.
 */
class DStarLitePlanner : public IPathPlanner {
  public:
    DStarLitePlanner();
    ~DStarLitePlanner() override = default;

    /**
     * @brief Supplies a grid map that may change over time.
     * @param grid Source occupancy or traversal grid used for planning.
     */
    void setMap(const map::Grid& grid) override;

    /**
     * @brief Configures the planner start location.
     * @param position Starting position expressed in planner coordinates.
     */
    void setStart(const PlannerPosition& position) override;

    /**
     * @brief Configures the planner goal location.
     * @param position Goal position expressed in planner coordinates.
     */
    void setGoal(const PlannerPosition& position) override;

    /**
     * @brief Computes a path honouring previously observed map changes.
     * @return Planned path containing waypoints and explored nodes.
     */
    PlannedPath computePath() override;

  private:
    struct NodeData {
        float g = std::numeric_limits<float>::infinity();
        float rhs = std::numeric_limits<float>::infinity();
    };

    struct Key {
        float k1 = std::numeric_limits<float>::infinity();
        float k2 = std::numeric_limits<float>::infinity();
    };

    struct QueueEntry {
        glm::ivec2 cell{};
        Key key{};
        std::size_t sequence = 0;
    };

    struct IVec2Hash {
        std::size_t operator()(const glm::ivec2& v) const noexcept {
            std::size_t h1 = std::hash<int>()(v.x);
            std::size_t h2 = std::hash<int>()(v.y);
            return h1 ^ (h2 << 1);
        }
    };

    struct QueueEntryCompare {
        bool operator()(const QueueEntry& lhs, const QueueEntry& rhs) const noexcept;
    };

    map::Grid m_map{};
    glm::ivec2 m_startCell{0, 0};
    glm::ivec2 m_goalCell{0, 0};
    glm::ivec2 m_lastStart{0, 0};
    bool m_hasMap = false;
    bool m_hasStart = false;
    bool m_hasGoal = false;
    bool m_initialized = false;

    float m_keyModifier = 0.0f;
    std::size_t m_queueSequence = 0;

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryCompare> m_openList;
    std::unordered_map<glm::ivec2, NodeData, IVec2Hash> m_nodeInfo;
    std::unordered_map<glm::ivec2, Key, IVec2Hash> m_openTable;
    std::unordered_set<glm::ivec2, IVec2Hash> m_pendingUpdates;
    std::vector<glm::ivec2> m_expandedNodes;

    void resetPlannerState();
    void initializePlanner();
    void applyPendingUpdates();
    void computeShortestPath();

    void updateVertex(const glm::ivec2& cell);
    std::vector<glm::ivec2> getNeighbors(const glm::ivec2& cell) const;
    float edgeCost(const glm::ivec2& from, const glm::ivec2& to) const;

    NodeData& dataFor(const glm::ivec2& cell);
    float g(const glm::ivec2& cell) const;
    float rhs(const glm::ivec2& cell) const;
    void setG(const glm::ivec2& cell, float value);
    void setRhs(const glm::ivec2& cell, float value);

    Key calculateKey(const glm::ivec2& cell) const;
    static bool keyLess(const Key& lhs, const Key& rhs);

    void pushOpen(const glm::ivec2& cell, const Key& key);
    bool isTrav(const glm::ivec2& cell) const;
    bool isValidStartGoal() const;
};

} // namespace planner
