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

    std::cout<<"here \n";
    
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
    
    std::queue<Node> open_list;
    // std::priority_queue<Node, std::vector<Node>, CompareNode> open_list;
    Node root(kings.size());
    for(int i=0;i<kings.size();i++) {
        root.paths[i].push_back(kings[i].start);
    }

    for (int i = 0; i < kings.size(); ++i) {
        std::vector<Position> existingPath = root.paths[i];
        int startTimeStep = 0;
        std::vector<Position> newPath = lowLevelSearch(i, 0, root, startTimeStep);
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
    int num_conflicts = 0;
    int agent1 = 0, agent2 = 0;
    int prevAgent1 = 0, prevAgent2 = 0;

    while (!open_list.empty()) {
        std::cout<<"iter number: "<<iter<<"\n";

        Node curr = open_list.front();
        

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
        auto [king2, timestep2, x2, y2] = conflict2;
        agent1 = king1;
        agent2 = king2;
        if(agent1==prevAgent1 && agent2==prevAgent2) {
            num_conflicts++;
        }
        else {
            num_conflicts = 0;
        }
        prevAgent1 = agent1;
        prevAgent2 = agent2;

        if (num_conflicts >= 50) {
            std::cerr << "Deadlock detected between agents " << agent1 << " and " << agent2 << ". Replanning...\n";
            
            // Replan both agents from scratch
            int startTimeStep = 0;
            // toFindConflict[0] = 
            curr.constraints[agent1].clear();
            curr.constraints[agent2].clear();
            std::vector<Position> newPath1 = lowLevelSearch(agent1, 0, curr, startTimeStep);
            std::vector<Position> newPath2 = lowLevelSearch(agent2, 0, curr, startTimeStep);
            curr.paths[agent1].clear();
            curr.paths[agent2].clear();

            curr.paths[agent1].push_back(kings[agent1].start);
            curr.paths[agent2].push_back(kings[agent2].start);
            for(int i=1;i<=newPath1.size();i++) {
                curr.paths[agent1].push_back(newPath1[i-1]);
            }
            for(int i=1;i<=newPath2.size();i++) {
                curr.paths[agent2].push_back(newPath2[i-1]);
            }
            

            curr.cost = calculateCost(curr.paths);
            // std::priority_queue<Node, std::vector<Node>, CompareNode>  empty;
            // std::swap(open_list, empty);

            open_list.push(curr);

            // Reset the repeated conflict count
            num_conflicts = 0;
            iter++;
            continue;
        }



        
        for(auto path: curr.paths) {
            if(!isPathValid(path)) {
                std::cerr<<"Invalid path before conflict resolution itself!\n";
            }
        }

        Node child1 = curr, child2 = curr;

        child1.constraints[king1].insert({timestep1, x1, y1});
        child2.constraints[king2].insert({timestep2, x2, y2});
        timestep1 = 0;
        timestep2 = 0;
        //calculate new path for child 1
        int startTimeStep1 = 0;
        std::vector<Position> newPath1 = lowLevelSearch(king1, 0, child1, startTimeStep1);
        if(!isPathValid(newPath1)) {
            std::cerr<<"Invalid path after conflict resolution!!! low level seaerch issue\n";
        }
        
        if(startTimeStep1!=timestep1-1 && startTimeStep1!=0) {
            std::cerr<<"Start time step not equal to timestep1\n";
        }

        if (!newPath1.empty()) {
            // if(!isPathValid(child1.paths[king1])) {
            //     std::cerr<<"dont know wtf is going on!\n";
            
            // }
            // child1.paths[king1].clear();
            // if(!isPathValid(curr.paths[king1])) {
            //     std::cerr<<"Invalid path before merge itself!\n";
            // }
            // for(int i=0;i<timestep1;i++) {
            //     child1.paths[king1].push_back(curr.paths[king1][i]);
            // }
            
            // if(!isPathValid(child1.paths[king1])) {
            //     std::cerr<<"Invalid path before merge until timestep "<<timestep1<<" itself!\n";
            // }
            // child1.paths[king1].insert(child1.paths[king1].end(), newPath1.begin(), newPath1.end());
            child1.paths[king1].clear();
            child1.paths[king1].push_back(kings[king1].start);
            for(int i=1;i<=newPath1.size();i++) {
                child1.paths[king1].push_back(newPath1[i-1]);
            }
            
        }

        bool validChild1 = true;
        
        if (!isPathValid(child1.paths[king1])) {
            std::cerr << "Invalid solution: King "<<king1<<" path has invalid moves after conflict resolution.\n";
            // return;
            validChild1 = false;
        }

        

            
        // }
        child1.cost = calculateCost(child1.paths);

        //calculate new path for child 2
        std::vector<Position> existingPath2 = child2.paths[king2];
        int startTimeStep2 = 0;
        std::vector<Position> newPath2 = lowLevelSearch(king2, 0, child2, startTimeStep2);
        if(startTimeStep2!=timestep2-1) {
            std::cerr<<"Start time step not equal to timestep2\n";
        }

        if (!newPath2.empty()) {
            if(!isPathValid(newPath2)) {
                std::cerr<<"Invalid path after conflict resolution!!! low level seaerch issue\n";
            }
            // if(!isPathValid(child2.paths[king2])) {
            //     std::cerr<<"dont know wtf is going on!\n";
            
            // }
            // child2.paths[king2].clear();
            // if(!isPathValid(curr.paths[king2])) {
            //     std::cerr<<"Invalid path before merge itself!\n";
            // }
            // for(int i=0;i<timestep2;i++) {
            //     child2.paths[king2].push_back(curr.paths[king2][i]);
            // }
            
            // if(!isPathValid(child2.paths[king2])) {
            //     std::cerr<<"Invalid path before merge until timestep "<<timestep1<<" itself!\n";
            // }
            // child2.paths[king2].insert(child2.paths[king2].end(), newPath2.begin(), newPath2.end());
            child2.paths[king2].clear();
            child2.paths[king2].push_back(kings[king2].start);
            for(int i=1;i<=newPath2.size();i++) {
                child2.paths[king2].push_back(newPath2[i-1]);
            }
        }



        child2.cost = calculateCost(child2.paths);
        int numConflictsCurr = calculateNumberOfConflicts(curr.paths);
        int numConflictsChild1 = calculateNumberOfConflicts(child1.paths);
        int numConflictsChild2 = calculateNumberOfConflicts(child2.paths);
        std::vector<Node> children = {curr, child1, child2};


        
        
        // bypass conflicts

        bool validChild2 = true;

        if (!isPathValid(child2.paths[king2])) {
            std::cerr << "Invalid solution: King "<<king2<<" path has invalid moves after conflict resolution.\n";
            // return;
            validChild2 = false;
        }

        if(validChild1) {
            if(numConflictsChild1<numConflictsCurr && child1.cost<=curr.cost ) {
                open_list = std::queue<Node>();
                // open_list = std::priority_queue<Node, std::vector<Node>, CompareNode>();
                open_list.push(child1);
                iter++;
                continue;
            }
        }


        

        if(validChild2) {
            if(numConflictsChild2<numConflictsCurr && child2.cost<=curr.cost) {
                open_list = std::queue<Node>();
                // open_list = std::priority_queue<Node, std::vector<Node>, CompareNode>();
                open_list.push(child2);
                iter++;
                continue;
            }
        }


        if(validChild1) {

            if (!child1.paths[king1].empty()) {
                open_list.push(child1);
            }   
        }

        if(validChild2) {
            if (!child2.paths[king2].empty()) {
                open_list.push(child2);
            }
        }

        
        
        // iter++;
        iter++;
        
    }

    noSolution = true;
    return false;
}


int ChessGame::calculateNumberOfConflicts( const std::vector<std::vector<Position>>& paths) {
    std::vector<std::vector<Position>> copyPaths = paths;
    int numberOfConflicts = 0;
    for (int k1 = 0; k1 < kings.size(); ++k1) {
        for (int k2 = 0; k2 < kings.size(); ++k2) {
            if(k1==k2) continue;
            std::vector<Position> path1 = paths[k1];
            std::vector<Position> path2 = paths[k2];

            // Find the maximum path length
            int max_length = std::max(path1.size(), path2.size());

            // Pad the shorter path with its last position
            while (path1.size() < max_length) {
                path1.push_back(path1.back());
            }
            while (path2.size() < max_length) {
                path2.push_back(path2.back());
            }
            for (int t = 0; t < max_length; ++t) {
                
                Position p1, p2;
                int t1 = t, t2 = t;


                
                if(k2>k1) {
                    t2 = t-1;
                }
                else t2 = t;
                
                if(t2==-1) {
                    
                    t2 = 0;
                }
                p1 = path1[t1];
                p2 = path2[t2];


                // Detect vertex conflict
                if (p1 == p2) {

                    numberOfConflicts++;
                }

                // Detect adjacency conflict
                std::vector<Position> adjacents = {
                    {p1.x + 1, p1.y}, {p1.x - 1, p1.y}, {p1.x, p1.y + 1}, {p1.x, p1.y - 1},
                    {p1.x + 1, p1.y + 1}, {p1.x - 1, p1.y - 1}, {p1.x + 1, p1.y - 1}, {p1.x - 1, p1.y + 1}
                };
                if (std::find(adjacents.begin(), adjacents.end(), p2) != adjacents.end()) {
                    
                    numberOfConflicts++;
                }
            }
        }
    }
    return numberOfConflicts;
}

int ChessGame::calculateCost(const std::vector<std::vector<Position>>& paths) {
    int cost = 0;
    for (int i = 0; i < paths.size(); ++i) {
        cost += paths[i].size();
    }
    return cost;
}

bool ChessGame::findConflicts( Node& node, const std::vector<King>& kings, std::tuple<int, int, int, int>& conflict1, std::tuple<int, int, int, int>& conflict2) {
    
    using ConflictPair = std::pair<std::tuple<int, int, int, int, int>, std::tuple<int, int, int, int, int>>; // Pair of conflicts
    auto cmp = [this](const ConflictPair& a, const ConflictPair& b) {
        // Compare minimum distances to goal
        int a_dist_min = std::min(std::get<4>(a.first), std::get<4>(a.second));
        int b_dist_min = std::min(std::get<4>(b.first), std::get<4>(b.second));
        return a_dist_min > b_dist_min; // Greater distance should be placed at the end
    };
    std::priority_queue<ConflictPair, std::vector<ConflictPair>, decltype(cmp)> conflictQueue(cmp);
    
    for (int k1 = 0; k1 < kings.size(); ++k1) {
        for (int k2 = 0; k2 < kings.size(); ++k2) {
            if(k1==k2) continue;
             int max_length = std::max(node.paths[k1].size(), node.paths[k2].size());

            // Pad the shorter path with its last position
            while (node.paths[k1].size() < max_length) {
                node.paths[k1].push_back(node.paths[k1].back());
            }
            while (node.paths[k2].size() < max_length) {
                if(node.paths[k2].size()==0) continue;
                node.paths[k2].push_back(node.paths[k2].back());
            }

            for (int t = 0; t < max_length; ++t) {
                Position p1, p2;
                int t1 = t, t2 = t;


                
                if(k2>k1) {
                    t2 = t-1;
                }
                else t2 = t;
                
                if(t2==-1) {
                    
                    t2 = 0;
                }
                

           
                p1 = node.paths[k1][t1];
                p2 = node.paths[k2][t2];

                int dist_to_goal1 = manhattanDistance(p1, kings[k1].target);
                int dist_to_goal2 = manhattanDistance(p2, kings[k2].target);
                int min_dist_to_goal = std::min(dist_to_goal1, dist_to_goal2);
                // Detect vertex conflict
                if (p1 == p2) {
                    // conflict1 = std::make_tuple(k1, t1, p1.x, p1.y);
                    // conflict2 = std::make_tuple(k2, t2, p2.x, p2.y);
                    // return true;
                    // if(isDeadlocked[0]!=-1) {
                    //     int deadlock1 = isDeadlocked[0];
                    //     int deadlock2 = isDeadlocked[1];
                    //     if((k1==deadlock1 && k2==deadlock2) || (k1==deadlock2 && k2==deadlock1)) {
                    //         continue;
                    //     }
                    // }
                    // conflictQueue.emplace(k1, t, p1.x, p1.y, min_dist_to_goal);
                    auto conflictPair = std::make_pair(std::make_tuple(k1, t, p1.x, p1.y, min_dist_to_goal), std::make_tuple(k2, t2, p2.x, p2.y, min_dist_to_goal));
                    conflictQueue.push(conflictPair);
                }

                // Detect adjacency conflict
                std::vector<Position> adjacents = {
                    {p1.x + 1, p1.y}, {p1.x - 1, p1.y}, {p1.x, p1.y + 1}, {p1.x, p1.y - 1},
                    {p1.x + 1, p1.y + 1}, {p1.x - 1, p1.y - 1}, {p1.x + 1, p1.y - 1}, {p1.x - 1, p1.y + 1}
                };
                if (std::find(adjacents.begin(), adjacents.end(), p2) != adjacents.end()) {
                    // conflict1 = std::make_tuple(k1, t1, p1.x, p1.y);
                    // conflict2 = std::make_tuple(k2, t2, p2.x, p2.y);
                    // return true;
                    auto conflictPair = std::make_pair(std::make_tuple(k1, t, p1.x, p1.y, min_dist_to_goal), std::make_tuple(k2, t2, p2.x, p2.y, min_dist_to_goal));
                    conflictQueue.push(conflictPair);
                }
            }
        }
    }

    if (!conflictQueue.empty()) {
        auto top_pair = conflictQueue.top();
        auto firstElem = top_pair.first;
        auto secondElem = top_pair.second;
        conflict1 = std::make_tuple(std::get<0>(firstElem), std::get<1>(firstElem), std::get<2>(firstElem), std::get<3>(firstElem));
        conflict2 = std::make_tuple(std::get<0>(secondElem), std::get<1>(secondElem), std::get<2>(secondElem), std::get<3>(secondElem));
        return true;
    }
    return false; // No conflicts found
}




std::vector<Position> ChessGame::lowLevelSearch(const int kingIndex, int startTime, const Node curr, int& startTimeStep) {
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

    while (!open_list.empty()) {
        auto [cost, timeStep, current] = open_list.top();
        open_list.pop();
        closed_list.insert(current);
        if (current == kings[kingIndex].target) {
            std::vector<Position> path;
            std::tuple<Position, int> current_key = {current, timeStep};
            while (!(current_key == start_key)) {
                Position prev = current;
                path.push_back(current);
                current_key = came_from[current_key];
                current = std::get<0>(current_key);
                startTimeStep = std::get<1>(current_key);
                if(!isValidKingMove(prev, current)) {
                    std::cerr<<"Invalid move\n";
                }
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
       

        for (auto& next : neighbors) {
            if (next.x < 0 || next.y < 0 || next.x >= size || next.y >= size) continue;
            if (grid[next.x][next.y] < 0) continue; // Skip blocked cells
            if (closed_list.find(next) != closed_list.end() ) {
                continue;
            }
            

            bool conflict = false;
            const auto& king_constraints = curr.constraints[kingIndex];
            Constraint c{timeStep+1, next.x, next.y};
            if (king_constraints.find(c) != king_constraints.end()) {
                conflict = true;
            }
            if (conflict) continue;


            int heuristic_cost = 10*kings[kingIndex].distance_map[next];
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



    for (const auto& king : kings) {
        if (!isPathValid(king.path)) {
            std::cerr << "Invalid solution: King path has invalid moves.\n";
            return;
        }
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


bool ChessGame::isValidKingMove(const Position& current, const Position& next) {
    int dx = std::abs(current.x - next.x);
    int dy = std::abs(current.y - next.y);
    return dx <= 1 && dy <= 1;
}

// Check if the path for a king is valid
bool ChessGame::isPathValid(const std::vector<Position>& path) {
    for (size_t i = 1; i < path.size(); ++i) {
        if (!isValidKingMove(path[i - 1], path[i])) {
            return false;
        }
    }
    return true;
}