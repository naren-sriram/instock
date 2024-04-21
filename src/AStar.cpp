#include <AStar.h>
std::vector<Position> AStar::search(const std::vector<std::vector<bool>>& board,
                                  const Position& start,
                                  const Position& goal,
                                  const std::set<Position>& constraints) {
    std::priority_queue<Vertex, std::vector<Vertex>, std::greater<Vertex>> openSet;
    std::map<Position, Position> cameFrom;
    std::map<Position, int> costSoFar;
    std::vector<Position> path;

    openSet.emplace(start, 0, 0);
    cameFrom[start] = start;
    costSoFar[start] = 0;

    while (!openSet.empty()) {
        Position current = openSet.top().position;
        int currentCost = openSet.top().cost;
        openSet.pop();

        if (current.x == goal.x && current.y == goal.y) {
            while (!(current == start)) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
            
        }

        // Adding movements including staying in the same position
        Position movements[9] = {
            {0, 0, 1}, {-1, -1, 1}, {-1, 0, 1}, {-1, 1, 1},
            {0, -1, 1}, {0, 1, 1}, {1, -1, 1}, {1, 0, 1}, {1, 1, 1}
        };

        for (const auto& move : movements) {
            Position next(current.x + move.x, current.y + move.y, current.time + move.time);
            if (next.x < 0 || next.y < 0 || next.x >= board.size() || next.y >= board.size() || board[next.x][next.y])
                continue;
            if (constraints.find(next) != constraints.end())
                continue;

            int newCost = currentCost + 1;
            if (!costSoFar.count(next) || newCost < costSoFar[next]) {
                costSoFar[next] = newCost;
                int priority = newCost + manhattanDistance(goal, next);
                openSet.emplace(next, priority, newCost);
                cameFrom[next] = current;
            }
        }
    }

    return path; // Return empty path if no path found
}