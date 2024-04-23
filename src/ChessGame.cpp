#include "ChessGame.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <fstream>

ChessGame::ChessGame(int n, const std::vector<std::vector<bool>>& initialGrid)
    : size(n), grid(n, std::vector<int>(n, -1)) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (initialGrid[i][j]) {
                grid[i][j] = n; // Use the grid size as the special value for permanently blocked cells
            }
        }
    }
    
}

void ChessGame::placeKings(const std::vector<King>& allKings) {
    int kingIndex = 0;  // Initialize king index to track which king is being placed
    for (const auto& king : allKings) {
        kings.push_back(king);
        
        kingIndex++;  // Increment the index for the next king
    }
    computeDistanceMaps();
    for(const auto& king: allKings) {
        if (grid[king.current.x][king.current.y] == -1) { // Ensure the spot is free before placing the king
            grid[king.current.x][king.current.y] = kingIndex;  // Place the king on the grid using their index
        }
        updateAdjacentCells(king.current, kingIndex, true);  // Block adjacent cells
    }
}

int ChessGame::heuristic(const Position& current, int kingIndex) const {
    if (kingIndex < 0 || kingIndex >= distanceMaps.size()) return std::numeric_limits<int>::max();
    return distanceMaps[kingIndex][current.x][current.y];
}


void ChessGame::findPaths() {
   bool allReached = false;
    while (!allReached) {
        allReached = true;
        int kingIndex = 0;
        for (auto& king : kings) {
            
            if (king.current == king.goal) continue; // Skip if already at goal
            allReached = false;
            updateAdjacentCells(king.current, kingIndex, false); 
            Position nextStep = findReevaluationMove(king, kingIndex);
            if (!(nextStep == king.current)) {  // A valid move is found
                king.closedList.insert(king.current);
                king.current = nextStep;    // Move king
                king.path.push_back(nextStep);
                updateAdjacentCells(king.current, kingIndex, true);
                king.waitCount = 0;         // Reset wait count on successful move
            } else {
                          // Increment wait count if the king has to wait
                if (king.waitCount > 0) {   // If king has waited more than once, force a re-evaluation for possible move
                    nextStep = findReevaluationMove(king, kingIndex);
                    if (!(nextStep == king.current)) {
                        king.closedList.insert(king.current);
                        king.current = nextStep;
                        king.path.push_back(nextStep);
                        updateAdjacentCells(king.current, kingIndex, true);
                        king.waitCount = 0; // Reset wait count on successful move
                    } else {
                        king.waitCount++; 
                        king.path.push_back(king.current);
                        updateAdjacentCells(king.current, kingIndex, true); // Continue waiting if no moves are possible
                    }
                } else {
                    king.waitCount++;
                    king.path.push_back(king.current);
                    updateAdjacentCells(king.current, kingIndex, true); // Log the wait by repeating the current position
                }
            }
            kingIndex++;
        }
    }
    std::cout<<"allreached: "<<allReached<<"\n";
}

Position ChessGame::findReevaluationMove(const King& king, int kingIndex) {
    std::vector<Position> neighbors = getNeighbors(king.current);
    neighbors.push_back(king.current);
    Position bestMove = king.current;
    int minHeuristic = std::numeric_limits<int>::max(); // Initialize with the maximum possible int value

    for (const Position& next : neighbors) {
        if (isFree(next) && (king.closedList.find(next)==king.closedList.end())) { // Ensure the next position is free
            int nextHeuristic = heuristic(next, kingIndex);
            if (nextHeuristic < minHeuristic) {
                minHeuristic = nextHeuristic;
                bestMove = next;
            }
        }
    }

    return bestMove; // Return the next best move based on heuristic cost
}



void ChessGame::updateAdjacentCells(Position pos, int kingIndex, bool block) {
    const int directions[9][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {0,0}, {-1, -1}, {1, -1}, {-1, 1}};
    for (auto& d : directions) {
        int nx = pos.x + d[0], ny = pos.y + d[1];
        if (nx >= 0 && nx < size && ny >= 0 && ny < size) {
            if(grid[nx][ny]==size) continue;
            if (block) {
                grid[nx][ny] = kingIndex; // Block with current king's index
            } else {
                // Only unblock if the cell was blocked by this king
                if (grid[nx][ny] == kingIndex) {
                    grid[nx][ny] = -1; // Mark as free
                }
            }
        }
    }
}

bool ChessGame::isFree(const Position& pos) const {
    // Check if the position is within grid boundaries
    if (pos.x < 0 || pos.x >= size || pos.y < 0 || pos.y >= size) {
        return false;  // Position is out of bounds
    }

    // Check if the cell is free or permanently blocked
    if (grid[pos.x][pos.y] == -1) {
        return true;  // Cell is free
    } 

    return false;  // Default case to handle dynamically blocked cells
}

void ChessGame::computeDistanceMaps() {
    distanceMaps.resize(kings.size()); // Ensure there's a map for each king
    for (size_t i = 0; i < kings.size(); ++i) {
        distanceMaps[i] = computeDijkstraMap(kings[i].goal);
    }
}

std::vector<std::vector<int>> ChessGame::computeDijkstraMap(const Position& goal) {
    std::vector<std::vector<int>> distMap(size, std::vector<int>(size, std::numeric_limits<int>::max()));
    std::queue<Position> queue;
    distMap[goal.x][goal.y] = 0;
    queue.push(goal);

    while (!queue.empty()) {
        Position current = queue.front();
        queue.pop();
        int currentDist = distMap[current.x][current.y];

        std::vector<Position> neighbors = getNeighbors(current);
        for (const auto& next : neighbors) {
            if (isFree(next) && currentDist + 1 < distMap[next.x][next.y]) {
                distMap[next.x][next.y] = currentDist + 1;
                queue.push(next);
            }
        }
    }

    return distMap;
}


// Position ChessGame::findNextStepAStar(const King& king) {
//     std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
//     std::map<Position, Position> cameFrom;
//     std::map<Position, int> costSoFar;
//     std::map<Position, int> heuristicMap;

//     Position start = king.current;
//     Position goal = king.goal;

//     openSet.push({start, heuristic(start, goal)});
//     cameFrom[start] = start;
//     costSoFar[start] = 0;
//     heuristicMap[start] = heuristic(start, goal);

//     while (!openSet.empty()) {
//         Node current = openSet.top();
//         openSet.pop();

//         if (current.pos == goal) {
//             // Goal reached; reconstruct the path to find the first step towards the goal
//             std::vector<Position> path;
//             for (Position pos = goal; !(pos == start); pos = cameFrom[pos]) {
//                 path.push_back(pos);
//             }
//             return path.back(); // Return the first step from the start towards the goal
//         }

//         for (const Position& next : getNeighbors(current.pos)) {
//             if (!isFree(next)) continue;

//             int newCost = costSoFar[current.pos] + 1;  // Assuming uniform cost
//             if (!costSoFar.count(next) || newCost < costSoFar[next]) {
//                 costSoFar[next] = newCost;
//                 int nextPriority = heuristic(next, goal);
//                 openSet.push({next, nextPriority});
//                 cameFrom[next] = current.pos;
//                 heuristicMap[next] = nextPriority;
//             }
//         }
//     }

//     return king.current;  // If no path is found, return current position as the "wait" move
// }





std::vector<Position> ChessGame::getNeighbors(const Position& pos) const {
    std::vector<Position> neighbors;
    const int dx[9] = {1, 1, 0, -1, -1, -1, 0, 1, 0};  // X offsets for 8 directions
    const int dy[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};  // Y offsets for 8 directions

    for (int i = 0; i < 8; i++) {
        int nx = pos.x + dx[i], ny = pos.y + dy[i];
        if (nx >= 0 && nx < size && ny >= 0 && ny < size) {  // Ensure within grid bounds
            neighbors.push_back({nx, ny});
        }
    }

    return neighbors;
}

void ChessGame::writePathsToFile( const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing.\n";
        return;
    }

    // Find the maximum path length to ensure correct interleaving
    size_t maxPathLength = 0;
    for (const auto& king : kings) {
        if (king.path.size() > maxPathLength) {
            maxPathLength = king.path.size();
        }
    }

    // Write moves in a round-robin fashion
    for (size_t step = 0; step < maxPathLength; ++step) {
        for (const auto& king : kings) {
            if (step < king.path.size()) {
                file << king.path[step].x << ", " << king.path[step].y << std::endl;
            } else if (!king.path.empty()) {
                // If no more steps, repeat the last position
                const auto& lastPos = king.path.back();
                file << lastPos.x << ", " << lastPos.y << std::endl;
            }
        }
    }
    file.close();
}

