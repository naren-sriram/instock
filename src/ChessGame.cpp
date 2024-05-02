#include <ChessGame.h>
#include <iostream>
#include <cmath>
#include <limits>
#include <fstream>

ChessGame::ChessGame(int n, const std::vector<std::vector<bool>>& initialGrid, std::vector<King>& allKings)
    : size(n), grid(n, std::vector<int>(n, 0)), kings(allKings) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (initialGrid[i][j]) {
                grid[i][j] = std::numeric_limits<int>::min(); // Use the grid size as the special value for permanently blocked cells
            }
        }
    }

    for(int i=0;i<kings.size();i++) {
        kings[i].distance_map = calculateDijkstraMap(kings[i].target,grid);
    }
    
}


std::unordered_map<Position, int, PositionHasher> ChessGame::calculateDijkstraMap(const Position& goal, const std::vector<std::vector<int>>& grid) {
    std::unordered_map<Position, int, PositionHasher> distance_map;
    std::priority_queue<std::pair<int, Position>, std::vector<std::pair<int, Position>>, std::greater<>> pq;
    
    pq.push({0, goal});
    distance_map[goal] = 0;

    while (!pq.empty()) {
        auto [distance, current] = pq.top();
        pq.pop();

        std::vector<Position> neighbors = {
            {current.x + 1, current.y}, {current.x - 1, current.y},
            {current.x, current.y + 1}, {current.x, current.y - 1},
            {current.x + 1, current.y + 1}, {current.x - 1, current.y - 1},
            {current.x + 1, current.y - 1}, {current.x - 1, current.y + 1}
        };

        for (const auto& neighbor : neighbors) {
            if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= grid.size() || neighbor.y >= grid[0].size()) {
                continue;
            }
            if (grid[neighbor.x][neighbor.y] < 0) continue; // Skip blocked cells

            int new_distance = distance + 1;
            if (distance_map.find(neighbor) == distance_map.end() || new_distance < distance_map[neighbor]) {
                distance_map[neighbor] = new_distance;
                pq.push({new_distance, neighbor});
            }
        }
    }

    return distance_map;
}


// Node comparison for priority queue




bool ChessGame::findPathsCBS() {
    std::priority_queue<Node, std::vector<Node>, NodeComparator> open_list;
    // std::queue<Node> open_list;
    
    Node root(kings.size());
    for(int i=0;i<kings.size();i++) {
        root.paths[i].push_back(kings[i].start);
    }
    for (int i = 0; i < kings.size(); ++i) {
        std::vector<Position> existingPath = root.paths[i];
        std::vector<Position> newPath = lowLevelSearch(i, 0, root);
        std::vector<Position> appendedPath;
        for(int i=0;i<newPath.size();i++) {
            existingPath.push_back(newPath[i]);
        }
        appendedPath = existingPath;
        root.paths[i].clear();
        root.paths[i] = appendedPath;
    }
    root.cost = calculateCost(root.paths);
    open_list.push(root);
    int iter = 0;
    while (!open_list.empty()) {
        std::cout<<"iter number: "<<iter<<"\n";
        Node curr = open_list.top();
        if(curr.cost==7) {
            // std::cout<<"cost is 7\n";
        }
        open_list.pop();
        std::tuple<int, int, int, int> conflict1;
        std::tuple<int, int, int, int> conflict2;
        if (!findConflicts(curr, kings, conflict1, conflict2)) {
            for(int i=0;i<kings.size();i++) {
                kings[i].path = curr.paths[i];
            }
            return true;
        }


        auto [king1, timestep1, x1, y1] = conflict1;
        auto [king2, timestep2, x2,y2] = conflict2;
        
        Node child1 = curr, child2 = curr;

        child1.constraints[king1].insert({timestep1, x1, y1});
        child2.constraints[king2].insert({timestep2, x2, y2});
        //calculate new path for child 1
        std::vector<Position> existingPath1 = child1.paths[king1];
        std::vector<Position> newPath1 = lowLevelSearch(king1, timestep1-1, child1);
        if(newPath1.size()!=0) {
            std::vector<Position> appendedPath1;
            for(int i=0;i<timestep1;i++) {
                appendedPath1.push_back(existingPath1[i]);
            }
            for(int k=0;k<newPath1.size();k++) {
                appendedPath1.push_back(newPath1[k]);
            }
            child1.paths[king1].clear();
            child1.paths[king1] = appendedPath1;
            child1.cost = calculateCost(child1.paths);
            // if(!findConflicts(child1, kings, conflict1, conflict2)) {
            //     for(int i=0;i<kings.size();i++) {
            //         kings[i].path = curr.paths[i];
            //     }
            //     return true;
            // }
        }

        //calculate new path for child 2
        std::vector<Position> existingPath2 = child2.paths[king2];
        std::vector<Position> newPath2 = lowLevelSearch(king2, timestep2-1, child2);
        if(newPath2.size()!=0) {
            std::vector<Position> appendedPath2;
            for(int i=0;i<timestep2;i++) {
                appendedPath2.push_back(existingPath2[i]);
            }
            for(int k=0;k<newPath2.size();k++) {
                appendedPath2.push_back(newPath2[k]);
            }
            child2.paths[king2].clear();
            child2.paths[king2] = appendedPath2;
            child2.cost = calculateCost(child2.paths);
            // if(!findConflicts(child2, kings, conflict1, conflict2)) {
            //     for(int i=0;i<kings.size();i++) {
            //         kings[i].path = curr.paths[i];
            //     }
            //     return true;
            // }
        }


         if (!child1.paths[king1].empty()) {
            open_list.push(child1);
        }
        if (!child2.paths[king2].empty()) {
            open_list.push(child2);
        }
        iter++;
    }

    noSolution = true;
    return false;
}


int ChessGame::calculateCost(const std::vector<std::vector<Position>>& paths) {
    int total_cost = 0;
    for (const auto& path : paths) {
        total_cost += path.size();
    }
    return total_cost;
}

bool ChessGame::findConflicts(const Node& node, const std::vector<King>& kings, std::tuple<int, int, int, int>& conflict1, std::tuple<int, int, int, int>& conflict2) {
    for (int k1 = 0; k1 < kings.size(); ++k1) {
        for (int k2 = 0; k2 < kings.size(); ++k2) {
            if(k1==k2) continue;
            for (int t = 0; t < node.paths[k1].size() && t < node.paths[k2].size(); ++t) {
                
                const auto& p1 = node.paths[k1][t];
                int t2 = 0;
                if(k2>k1) {
                    t2 = t-1;
                }
                else t2 = t;
                Position p2 = Position(0,0);
                if(t2==-1) {
                    
                    t2 = 0;
                }
                p2 = node.paths[k2][t2];

                // Detect vertex conflict
                if (p1 == p2) {
                    conflict1 = std::make_tuple(k1, t, p1.x, p1.y);
                    conflict2 = std::make_tuple(k2, t2, p2.x, p2.y);
                    return true;
                }

                // Detect adjacency conflict
                std::vector<Position> adjacents = {
                    {p1.x + 1, p1.y}, {p1.x - 1, p1.y}, {p1.x, p1.y + 1}, {p1.x, p1.y - 1},
                    {p1.x + 1, p1.y + 1}, {p1.x - 1, p1.y - 1}, {p1.x + 1, p1.y - 1}, {p1.x - 1, p1.y + 1}
                };
                if (std::find(adjacents.begin(), adjacents.end(), p2) != adjacents.end()) {
                    conflict1 = std::make_tuple(k1, t, p1.x, p1.y);
                    conflict2 = std::make_tuple(k2, t2, p2.x, p2.y);
                    return true;
                }
            }
        }
    }
    return false; // No conflicts found
}


std::vector<Position> ChessGame::lowLevelSearch(int kingIndex, int startTime, Node curr) {
    std::cout<<"Low level search for king: "<<kingIndex<<"\n";
    std::priority_queue<std::tuple<int, int, Position>, std::vector<std::tuple<int, int, Position>>, std::greater<>> open_list;
    std::unordered_map<std::tuple<Position, int>, int, PositionTimeHasher> cost_map;
    std::unordered_map<std::tuple<Position, int>, std::tuple<Position, int>, PositionTimeHasher> came_from;
    std::unordered_set<Position, PositionHasher> closed_list;
    Position start = Position(0,0);
    start = curr.paths[kingIndex][startTime];
    std::tuple<Position, int> start_key = {start, startTime};
    open_list.push({0, startTime, start});
    cost_map[{start, startTime}] = 0;
    // for(int i=0;i<=startTime;i++) {
    //     closed_list.insert(curr.paths[kingIndex][i]);
    // }
    // iterate through king's current path. 
    // if 2 next positions are same, increase wait count by 1
    // kings[kingIndex].waitCount = 0;
    // for (int i = 0; i < curr.paths[kingIndex].size()-1; ++i) {
    //     closed_list.insert(curr.paths[kingIndex][i]);
    // }
    while (!open_list.empty()) {
        auto [cost, timeStep, current] = open_list.top();
        open_list.pop();
        closed_list.insert(current);
        if (current == kings[kingIndex].target) {
            std::vector<Position> path;
            std::tuple<Position, int> current_key = {current, timeStep};
            while (!(current_key == start_key)) {
                path.push_back(current);
                current_key = came_from[current_key];
                current = std::get<0>(current_key);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        std::vector<Position> neighbors = {
                current,
                {current.x + 1, current.y}, {current.x - 1, current.y},
                {current.x, current.y + 1}, {current.x, current.y - 1},
                {current.x + 1, current.y + 1}, {current.x - 1, current.y - 1},
                {current.x + 1, current.y - 1}, {current.x - 1, current.y + 1}
            };
        // if(kings[kingIndex].waitCount>=1) {
        //     neighbors.resize(8);
        //     neighbors = {
        //         {current.x + 1, current.y}, {current.x - 1, current.y},
        //         {current.x, current.y + 1}, {current.x, current.y - 1},
        //         {current.x + 1, current.y + 1}, {current.x - 1, current.y - 1},
        //         {current.x + 1, current.y - 1}, {current.x - 1, current.y + 1}
        //     };
        // }
        // else {
        //    neighbors = {
        //         current,
        //         {current.x + 1, current.y}, {current.x - 1, current.y},
        //         {current.x, current.y + 1}, {current.x, current.y - 1},
        //         {current.x + 1, current.y + 1}, {current.x - 1, current.y - 1},
        //         {current.x + 1, current.y - 1}, {current.x - 1, current.y + 1}
        //     };
        // }

        // for all neighbors, 
        // if all of them are either blocked, or in closed list or adjacent to otehr kings
        // then push current position with cost+1 to open list, timeStep++ and continue
        // else proceed below


        // bool all_neighbors_blocked = true;
        // for (auto& next : neighbors) {
        //     if (next.x < 0 || next.y < 0 || next.x >= size || next.y >= size) continue;
        //     if (grid[next.x][next.y] < 0) continue; // Skip blocked cells

        //     // Check if it's adjacent to other kings
        //     bool adjacent_to_other_king = false;
        //     for (int k = 0; k < kings.size(); ++k) {
                
        //         int t2 = 0;
        //         if(k>kingIndex) {
        //             t2 = timeStep-1;
        //         }
        //         else t2 = timeStep;
        //         Position p2 = Position(0,0);
        //         if(t2==-1) {                   
        //             t2 = 0;
        //         }
        //         if(t2==0 && curr.paths[k].size()==0) {
        //             all_neighbors_blocked = false;
        //             break;
        //         }
        //         if(curr.paths[k].size()>=t2) {
        //             p2 = curr.paths[k][t2];
        //         }
        //         else { 
        //             all_neighbors_blocked = false;
        //             break;
        //         }
        //         if (k != kingIndex && std::abs(p2.x - next.x) <= 1 && std::abs(p2.y - next.y) <= 1) {
        //             adjacent_to_other_king = true;
        //             break;
        //         }
        //     }

        //     // If the position isn't in the closed list, isn't adjacent to other kings, or isn't blocked, we can move there
        //     if (closed_list.find(next) == closed_list.end() && !adjacent_to_other_king) {
        //         all_neighbors_blocked = false;
        //         break;
        //     }
        // }

        // if (all_neighbors_blocked) {
        //     if(curr.paths[kingIndex].size()>1) {
        //         int t = timeStep+1;
        //         open_list.push({cost + 1, t, current});
                
        //         continue;
        //     }
        // }

        for (auto& next : neighbors) {
            if (next.x < 0 || next.y < 0 || next.x >= size || next.y >= size) continue;
            if (grid[next.x][next.y] < 0) continue; // Skip blocked cells
            if (closed_list.find(next) != closed_list.end() && !(next==current)) {
                continue;
            }
            

            bool conflict = false;
            const auto& king_constraints = curr.constraints[kingIndex];
            Constraint c{timeStep+1, next.x, next.y};
            if (king_constraints.find(c) != king_constraints.end()) {
                conflict = true;
            }
            if (conflict) continue;


            int heuristic_cost = 100*kings[kingIndex].distance_map[next];
            // int heuristic_cost = manhattanDistance(next, kings[kingIndex].target);
            int new_cost = cost + 1 + heuristic_cost;
            
            std::tuple<Position, int> next_key = {next, timeStep + 1};

            if (cost_map.find(next_key) == cost_map.end() || new_cost < cost_map[next_key]) {
                cost_map[next_key] = new_cost;
                came_from[next_key] = {current, timeStep};
                open_list.push({new_cost, timeStep + 1, next});
            }
        }
    }
    return {}; // No valid path found
}

int ChessGame::manhattanDistance(const Position& a, const Position& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
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