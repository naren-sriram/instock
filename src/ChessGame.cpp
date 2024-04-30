#include "ChessGame.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <fstream>

ChessGame::ChessGame(int n, const std::vector<std::vector<bool>>& initialGrid)
    : size(n), grid(n, std::vector<int>(n, 0)) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (initialGrid[i][j]) {
                grid[i][j] = std::numeric_limits<int>::min(); // Use the grid size as the special value for permanently blocked cells
            }
        }
    }
    
}

void ChessGame::placeKings( std::vector<King>& allKings) {
    int kingIndex = 0;  // Initialize king index to track which king is being placed
    for (const auto& king : allKings) {
        kings.push_back(king);
        
        kingIndex++;  // Increment the index for the next king
    }
    kingIndex = 0;
    computeDistanceMaps();
    
    for( auto& king: kings) {
        king.path.push_back(king.current);
        updateAdjacentCells(king.current, true);  
        kingIndex++;// Block adjacent cells
    }
    kingIndex = 0;
}

int ChessGame::heuristic(const Position& current, int kingIndex) const {
    if (kingIndex < 0 || kingIndex >= distanceMaps.size()) return std::numeric_limits<int>::max();
    return distanceMaps[kingIndex][current.x][current.y];
}


bool ChessGame::findPaths() {
   bool allReached = false;
    while (!allReached && !noSolution) {
        allReached = true;
        int kingIndex = 0;
        for (auto& king : kings) {
            
            if (king.current == king.goal) {
                std::cout<<"king "<<kingIndex<<" reached goal! \n";
                kingIndex++;
                continue; 
            }// Skip if already at goal
            allReached = false;
            updateAdjacentCells(king.current, false); 
            Position nextStep(0,0);
            if(king.waitCount==0)
                nextStep = findReevaluationMove(king, kingIndex, false);
            else 
                nextStep = findReevaluationMove(king, kingIndex, true);
            if (!(nextStep == king.current)) {  // A valid move is found
                king.closedList.insert(king.current);
                // std::cout<<"king "<<kingIndex<<" moved from: "<<king.current.x<<", "<<king.current.y<<" : TO : "<<nextStep.x<<", "<<nextStep.y<<"\n";
                king.current = nextStep;    // Move king
                king.path.push_back(nextStep);
                updateAdjacentCells(nextStep, true);
                king.waitCount = 0;         // Reset wait count on successful move
            } else {
                          // Increment wait count if the king has to wait
                if (king.waitCount > 0) {   // If king has waited more than once, force a re-evaluation for possible move
                    nextStep = findReevaluationMove(king, kingIndex, true);
                    if (!(nextStep == king.current)) {
                        king.closedList.insert(king.current);
                        // std::cout<<"king "<<kingIndex<<" moved from: "<<king.current.x<<", "<<king.current.y<<" : TO : "<<nextStep.x<<", "<<nextStep.y<<"\n";
                        king.current = nextStep;
                        king.path.push_back(nextStep);
                        updateAdjacentCells(nextStep, true);
                        king.waitCount = 0; // Reset wait count on successful move
                    } else {
                        king.waitCount++; 
                        // std::cout<<"king "<<kingIndex<<" waiting at "<<king.current.x<<", "<<king.current.y<<" for "<<king.waitCount<<" timesteps.\n";
                        king.path.push_back(king.current);
                        updateAdjacentCells(nextStep, true);
                    }
                } else {
                    king.waitCount++;
                    // std::cout<<"king "<<kingIndex<<" waiting at "<<king.current.x<<", "<<king.current.y<<" for "<<king.waitCount<<" timesteps.\n";
                    king.path.push_back(king.current);
                    updateAdjacentCells(nextStep, true);

                }
            }
            kingIndex++;
        }
    }
    if(noSolution) std::cout<<"No solution found. \n";
    else std::cout<<"allreached: "<<allReached<<"\n";
    return !noSolution;
    
}

Position ChessGame::findReevaluationMove(const King& king, int kingIndex, bool isWaiting) {
    std::priority_queue<HeuristicNode, std::vector<HeuristicNode>, std::greater<HeuristicNode>> minHeap;
    std::vector<Position> neighbors = getNeighbors(king.current);  
    
    // Populate the priority queue with neighbors and their heuristic values
    for (const Position& next : neighbors) {  
        int nextHeuristic = heuristic(next, kingIndex);
        minHeap.push({next, nextHeuristic});
    }
        

    // Return the first available best move based on heuristic cost
    while (!minHeap.empty()) {
        HeuristicNode top = minHeap.top();
        minHeap.pop();
        if (isFree(top.pos) )  { 
            if(king.waitCount>5) {
                kings[kingIndex].crossedLongWait++;
                if(king.crossedLongWait>100) noSolution = true;
                return top.pos;
            }
                
            else if (king.closedList.find(top.pos) == king.closedList.end())
                return top.pos;
        }
    }

    return king.current;
}




void ChessGame::updateAdjacentCells(Position pos, bool increment) {
    // Directions for the eight surrounding cells in an 8-connected grid
    const int directions[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {-1, -1}, {1, -1}, {-1, 1}
    };

    for (auto& direction : directions) {
        int nx = pos.x + direction[0], ny = pos.y + direction[1];
        if (nx >= 0 && nx < size && ny >= 0 && ny < size) {
            // Only modify the grid if the target cell is not permanently blocked
            if (grid[nx][ny] != std::numeric_limits<int>::min()) {
                grid[nx][ny] += (increment ? 1 : -1);
                
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
    if (grid[pos.x][pos.y] == 0) {
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




std::vector<Position> ChessGame::getNeighbors(const Position& pos) const {
    std::vector<Position> neighbors;
    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};  // X offsets for 8 directions
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};  // Y offsets for 8 directions

    for (int i = 0; i < 8; i++) {
        int nx = pos.x + dx[i], ny = pos.y + dy[i];
        if (nx >= 0 && nx < size && ny >= 0 && ny < size) {  // Ensure within grid bounds
            neighbors.push_back({nx, ny});
        }
    }

    return neighbors;
}


void ChessGame::writePathsToFile(const std::string& filename) {
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

    // Write moves in a round-robin fashion with consecutive positions
    for (size_t step = 0; step < maxPathLength - 1; step++) {
        for (const auto& king : kings) {
            if (step < king.path.size() - 1) {
                file << king.path[step].x << ", " << king.path[step].y << ", "
                     << king.path[step + 1].x << ", " << king.path[step + 1].y << std::endl;
            } else {
                // If no more steps available, repeat the last known position
                const auto& lastPos = king.path.back();
                file << lastPos.x << ", " << lastPos.y << ", "
                     << lastPos.x << ", " << lastPos.y << std::endl;
            }
        }
    }
    file.close();
}