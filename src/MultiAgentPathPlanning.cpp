#include <MultiAgentPathPlanning.h>


MultiAgentPathPlanning::MultiAgentPathPlanning(const std::vector<King>& kings, const std::vector<std::vector<bool>>& board, int size)
    : kings(kings), board(board), boardSize(size), allPaths(kings.size()) {}

bool MultiAgentPathPlanning::findPaths() {
    // Initial path setup
    for (int i = 0; i < kings.size(); ++i) {
        allPaths[i].push_back(kings[i].start);
       
    }
    for(int i=0;i<kings.size(); ++i) {
         if (!planPathForKing(i, i + 1)) {
            std::cout << "Initial planning failed for king " << i + 1 << std::endl;
            return false;
        }
    }

    // Validate and replan if necessary
    std::vector<int> kingsNeedingReplan = validatePaths();
    while (!kingsNeedingReplan.empty()) {
        std::cout << "Conflicts detected, replanning for affected kings..." << std::endl;
        for (int kingIndex : kingsNeedingReplan) {
            if (!replanPath(kingIndex)) {
                std::cout << "Replanning failed for king " << kingIndex + 1 << std::endl;
                return false;
            }
        }
        kingsNeedingReplan = validatePaths(); // Re-check for conflicts after replanning
    }

    return true;  // All paths are valid and no conflicts exist
}


std::vector<int> MultiAgentPathPlanning::validatePaths() {
    std::vector<int> kingsNeedingReplan;
    for (int i = 0; i < kings.size(); ++i) {
        for (int j = i + 1; j < kings.size(); ++j) {
            for (int t = 1; t < std::max(allPaths[i].size(), allPaths[j].size()); ++t) {
                if (t < allPaths[i].size() && t < allPaths[j].size()) {
                    Position pos1 = allPaths[i][t];
                    Position pos2 = allPaths[j][t-1];
                    if (std::abs(pos1.x - pos2.x) <= 1 && std::abs(pos1.y - pos2.y) <= 1) {
                        kingsNeedingReplan.push_back(i);
                        kingsNeedingReplan.push_back(j);
                    }
                }
            }
        }
    }
    return kingsNeedingReplan;
}


bool MultiAgentPathPlanning::replanPath(int kingIndex) {
    // Clear the current path from the point of conflict
    if (!allPaths[kingIndex].empty()) {
        allPaths[kingIndex].erase(allPaths[kingIndex].begin() + 1, allPaths[kingIndex].end());
    }

    // Attempt to replan from the last valid position
    int timeOffset = kingIndex + 1;  // Starting time offset for this king

    return planPathForKing(kingIndex, timeOffset);
}




void MultiAgentPathPlanning::outputSolution() {
    // Output the paths for all kings as described
    for (int i = 0; i < allPaths.size(); ++i) {
        std::cout << "King " << (i + 1) << " path: ";
        for (const auto& pos : allPaths[i]) {
            std::cout << pos.x << ", " << pos.y << " -> ";
        }
        std::cout << std::endl;
    }
}



bool MultiAgentPathPlanning::planPathForKing(int kingIndex, int timeOffset) {
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> frontier;
    std::map<Position, Position> cameFrom;
    std::map<Position, int> costSoFar;
    std::set<Position> visited;  // Set to track visited nodes

    Position start = allPaths[kingIndex][0];
    Position goal = kings[kingIndex].goal;

    frontier.push({start, 0, heuristic(start, goal)});
    cameFrom[start] = start;
    costSoFar[start] = 0;

    while (!frontier.empty()) {
        auto current = frontier.top().pos;
        frontier.pop();

        if (visited.count(current)) continue;  // Skip processing if already visited
        visited.insert(current);  // Mark current node as visited

        if (current.x == goal.x && current.y == goal.y) {
            goal.time = current.time;
            rebuildPath(kingIndex, start, goal, cameFrom);
            updateKingPaths(kingIndex);  // Update paths for other kings based on this new path
            return true;
        }

        for (const auto& next : getNeighbors(current, timeOffset)) {
            if (isBlocked(next) || isOccupiedByOtherKings(next, kingIndex, current.time + 1) || visited.count(next))
                continue;

            int newCost = costSoFar[current] + 1;
            if (!costSoFar.count(next) || newCost < costSoFar[next]) {
                costSoFar[next] = newCost;
                int priority = newCost + heuristic(next, goal);
                frontier.push({next, newCost, priority});
                cameFrom[next] = current;
            }
        }
    }

    return false;  // No path found
}


void MultiAgentPathPlanning::updateKingPaths(int updatedKingIndex) {
    // Replan for all kings after updatedKingIndex
    for (int i = updatedKingIndex + 1; i < kings.size(); ++i) {
        if (!planPathForKing(i, i + 1)) {  // Adjust the starting offset for each king
            std::cout << "Replanning for king " << i + 1 << std::endl;
            planPathForKing(i, i + 1);
        }
    }
}



bool MultiAgentPathPlanning::isOccupiedByOtherKings(const Position& pos, int kingIndex, int timeStep) {
    // Check other kings' paths for conflicts at the given time step
    for (int i = 0; i < kings.size(); ++i) {
        if (i != kingIndex) {
            for (const auto& p : allPaths[i]) {
                if (p.time == timeStep && (p.x == pos.x && p.y == pos.y)) {
                    return true;  // Block position occupied by other king at the same time
                }
                // Check for adjacency including diagonals
                if (p.time == timeStep && std::abs(p.x - pos.x) <= 1 && std::abs(p.y - pos.y) <= 1) {
                    return true;  // Block if within one cell of another king
                }
            }
        }
    }

    // Block initial positions and their adjacent cells for the first move of each king based on their move order in round-robin
    if (timeStep == kingIndex + 1) {
        for (int i = 0; i < kings.size(); ++i) {
            if (i != kingIndex) {
                if (std::abs(kings[i].start.x - pos.x) <= 1 && std::abs(kings[i].start.y - pos.y) <= 1) {
                    return true;  // Block the starting position and all adjacent positions of any other king on their initial move
                }
            }
        }
    }
    return false;
}




// Heuristic function for A* (Manhattan distance)
int MultiAgentPathPlanning::heuristic(const Position& a, const Position& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// Check if the position is within the bounds and not blocked
bool MultiAgentPathPlanning::isBlocked(const Position& pos) {
    // Check if position is out of bounds or statically blocked
    if (pos.x < 0 || pos.x >= boardSize || pos.y < 0 || pos.y >= boardSize) {
        return true; // Out of bounds
    }
    if (board[pos.x][pos.y]) {
        return true; // Statically blocked
    }
    return false;
}



// Rebuild the path from start to goal
void MultiAgentPathPlanning::rebuildPath(int kingIndex, const Position& start, const Position& goal, const std::map<Position, Position>& cameFrom) {
    Position current = goal;
    std::vector<Position> path;
    while (!(current== start)) {
        path.push_back(current);
        current = cameFrom.at(current);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    allPaths[kingIndex] = path;
}

// Get neighbors of a position (including staying in place)
std::vector<Position> MultiAgentPathPlanning::getNeighbors(const Position& pos, int timeOffset) {
    std::vector<Position> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            neighbors.push_back({pos.x + dx, pos.y + dy, pos.time + timeOffset});
        }
    }
    return neighbors;
}