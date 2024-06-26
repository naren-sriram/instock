#include <ChessGame.h>
#include <iostream>
#include <cmath>
#include <limits>
#include <fstream>

ChessGame::ChessGame(int n, const std::vector<std::vector<bool>>& initialGrid, std::vector<King>& allKings)
    : size(n), grid(n, std::vector<int>(n, 0)), kings(allKings), traffic(n, std::vector<int>(n, 0)){
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

// void ChessGame::initializeTrafficTable(Node& node, int rows, int cols) {


//     // Initialize the traffic for each king with zeroes
//     for (int kingId = 0; kingId < kings.size(); ++kingId) {
//         // Create a grid of size rows x cols initialized to 0
//         std::vector<std::vector<int>> grid(rows, std::vector<int>(cols, 0));
//         node.traffic[kingId] = grid;
//     }

// }


bool ChessGame::findPathsCBS() {
    
    std::queue<Node> open_list;
    // std::priority_queue<Node, std::vector<Node>, CompareNode> open_list;
    Node root(kings.size());

    std::unordered_set<std::pair<std::tuple<int, int, int, int>, std::tuple<int, int, int, int>>, ConflictHasher> conflict_tracker;
    for(int i=0;i<kings.size();i++) {
        root.paths[i].push_back(kings[i].start);
    }
    // initializeTrafficTable(root, size, size);
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
    
    for(int i=0;i<kings.size();i++) {
        int conflicts = calculateNumberOfConflictsPerAgent(root, root.paths, i);
        root.numConflicts[i] = conflicts;
        root.totalConflicts += conflicts;
    }
    open_list.push(root);
    int iter = 0;
    int num_conflicts = 0;
    int agent1 = 0, agent2 = 0;
    int prevAgent1 = 0, prevAgent2 = 0;

    while (!open_list.empty()) {
        std::cout<<iter<<"\n";

        Node curr = open_list.front();
        

        open_list.pop();
        stats.expandedNodes++;
        std::tuple<int, int, int, int> conflict1 = {-1,-1,-1,-1};
        std::tuple<int, int, int, int> conflict2 = {-1,-1,-1,-1};
        int cardinality = -1;
        
        //
        // findCardinalConflict(curr, kings, conflict1, conflict2, cardinality)
        if (!findConflicts(curr, kings, conflict1, conflict2)) {
            for(int i=0;i<kings.size();i++) {
                kings[i].path = curr.paths[i];
            }
            stats.iterations = iter;
            stats.cost = curr.cost;
            return true;
        }
        // updateTraffic(curr.paths);
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

        if(king1==-1 && king2==-1) continue;


        Node child1 = curr, child2 = curr;
        // std::cout<<"conflict between kings "<<king1<<" and "<<king2<<" at "<<x1<<","<<y1<<" and "<<x2<<","<<y2<<" at time "<<timestep1<<" and "<<timestep2<<"\n";
        if(king1!=-1)
            child1.constraints[king1].insert({timestep1, x1, y1});
        if(king2!=-1)
            child2.constraints[king2].insert({timestep2, x2, y2});
        timestep1 = 0;
        timestep2 = 0;



        //calculate new path for child 1
        int startTimeStep1 = 0;
        std::vector<Position> newPath1;
        
        if(king1!=-1)
            newPath1 = lowLevelSearch(king1, 0, child1, startTimeStep1);
 

        if(newPath1.size()==0 && king1!=-1) {
            std::cerr<<"Empty path for king "<<king1<<"\n";
        }

        if (!newPath1.empty()) {
           
            traffic[x1][y1]--;
            int pathSize = child1.paths[king1].size();
            child1.paths[king1].clear();
            child1.paths[king1].push_back(kings[king1].start);
            for(int i=1;i<=newPath1.size();i++) {
                child1.paths[king1].push_back(newPath1[i-1]);
            }
            if(child1.paths[king1].size()<pathSize) {
                while(child1.paths[king1].size()<pathSize) {
                    child1.paths[king1].push_back(child1.paths[king1].back());
                }
            }
            
        }
        else {
            std::cerr<<"Empty path for king "<<king1<<"\n";
        }



        
        if(!newPath1.empty())
            child1.cost = calculateCost(child1.paths);

        //calculate new path for child 2
        int startTimeStep2 = 0;
        std::vector<Position> newPath2;


        if(king2!=-1)
            newPath2 = lowLevelSearch(king2, 0, child2, startTimeStep2);


        if (!newPath2.empty()) {
            // if(!isPathValid(newPath2)) {
            //     std::cerr<<"Invalid path after conflict resolution!!! low level seaerch issue\n";
            // }
            traffic[x2][y2]--;
            int pathSize = child2.paths[king2].size();
            child2.paths[king2].clear();
            child2.paths[king2].push_back(kings[king2].start);
            for(int i=1;i<=newPath2.size();i++) {
                child2.paths[king2].push_back(newPath2[i-1]);
            }
            if(child2.paths[king2].size()<pathSize) {
                
                while(child2.paths[king2].size()<pathSize) {
                    child2.paths[king2].push_back(child2.paths[king2].back());
                } 
            }
        }

        else {
            std::cerr<<"Empty path for king "<<king2<<"\n";
        }

        if(!newPath1.empty() && !newPath2.empty()) {
            //insert into conflict closed list
            conflict_tracker.insert({conflict1, conflict2});
        }
        if(!newPath2.empty())
            child2.cost = calculateCost(child2.paths);

        // std::fill(traffic.begin(), traffic.end(), std::vector<int>(size, 0));
        // int numConflictsCurr = calculateNumberOfConflicts(curr.paths);
        // int numConflictsChild1 = calculateNumberOfConflicts(child1.paths);
        // int numConflictsChild2 = calculateNumberOfConflicts(child2.paths);
        // child1.totalConflicts = numConflictsChild1;
        // child2.totalConflicts = numConflictsChild2;
        // curr.totalConflicts = numConflictsCurr;

        if(!newPath1.empty()) {
            curr.totalConflicts -= curr.numConflicts[king1];
            child1.totalConflicts -= curr.numConflicts[king1];
            child1.numConflicts[king1] = calculateNumberOfConflictsPerAgent(child1, child1.paths, king1);
            curr.totalConflicts += child1.numConflicts[king1];
            child1.totalConflicts += child1.numConflicts[king1];
        }
        
        if(!newPath2.empty()) {
            curr.totalConflicts -= curr.numConflicts[king2];
            child2.totalConflicts -= curr.numConflicts[king2];
            child2.numConflicts[king2] = calculateNumberOfConflictsPerAgent(child2, child2.paths, king2);
            curr.totalConflicts += child2.numConflicts[king2];
            child2.totalConflicts += child2.numConflicts[king2];
        }
       
        
        // bypass conflicts

        if(king1!=-1) {
            if(newPath1.empty()) {
                std::cerr<<"Empty path for king "<<king1<<"\n";
            }
            else {
                 if(child1.totalConflicts<curr.totalConflicts && child1.cost<=curr.cost ) {
                    open_list = std::queue<Node>();
                    // open_list = std::priority_queue<Node, std::vector<Node>, CompareNode>();

                    open_list.push(child1);
                    stats.generatedNodes++;
                    iter++;
                    continue;
                }
            }
           
        }


        
        if(king2!=-1) {
            if(newPath2.empty()) {
                std::cerr<<"Empty path for king "<<king2<<"\n";
            }
            else {
                if(child2.totalConflicts<curr.totalConflicts && child2.cost<=curr.cost) {
                    open_list = std::queue<Node>();
                    // open_list = std::priority_queue<Node, std::vector<Node>, CompareNode>();
                    open_list.push(child2);
                    stats.generatedNodes++;
                    iter++;
                    continue;
                }
            }
            
        }


        
        

        if(king1!=-1 && !newPath1.empty()) {
            // if(child1.cost<=curr.cost) {
            //     open_list = std::queue<Node>();
            //     open_list.push(child1);
            //     stats.generatedNodes++;
            //     iter++;
            //     continue;
            // }
            open_list.push(child1);
            stats.generatedNodes++;
        }
        if(king2!=-1 && !newPath2.empty()) {
            // if(child2.cost<=curr.cost) {
            //     open_list = std::queue<Node>();
            //     open_list.push(child2);
            //     stats.generatedNodes++;
            //     iter++;
            //     continue;
            // }
            open_list.push(child2);
            stats.generatedNodes++;
        }
        // iter++;
        iter++;
        
    }

    noSolution = true;
    return false;
}


int ChessGame::calculateNumberOfConflictsPerAgent(Node& node, const std::vector<std::vector<Position>>& paths, int k1) {
    std::vector<std::vector<Position>> copyPaths = paths;
    int numberOfConflicts = 0;
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
            for (int t = 1; t < max_length; ++t) {
                
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
                    // node.traffic[k1][p1.x][p1.y]++;
                    // node.traffic[k1][p2.x][p2.y]++;
                    // node.traffic[k2][p1.x][p1.y]++;
                    // node.traffic[k2][p2.x][p2.y]++;
                    traffic[p1.x][p1.y]++;
                    traffic[p2.x][p2.y]++;
                }

                // Detect adjacency conflict
                std::vector<Position> adjacents = {
                    {p1.x + 1, p1.y}, {p1.x - 1, p1.y}, {p1.x, p1.y + 1}, {p1.x, p1.y - 1},
                    {p1.x + 1, p1.y + 1}, {p1.x - 1, p1.y - 1}, {p1.x + 1, p1.y - 1}, {p1.x - 1, p1.y + 1}
                };
                if (std::find(adjacents.begin(), adjacents.end(), p2) != adjacents.end()) {
                    
                    numberOfConflicts++;
                    // node.traffic[k1][p1.x][p1.y]++;
                    // node.traffic[k1][p2.x][p2.y]++;
                    // node.traffic[k2][p1.x][p1.y]++;
                    // node.traffic[k2][p2.x][p2.y]++;
                    traffic[p1.x][p1.y]++;
                    traffic[p2.x][p2.y]++;
                }
            }
        }
    return numberOfConflicts;
}

// int ChessGame::updateTraffic( const std::vector<std::vector<Position>>& paths) {
//     std::vector<std::vector<Position>> copyPaths = paths;
//     int numberOfConflicts = 0;
//     for (int k1 = 0; k1 < kings.size(); ++k1) {
//         for (int k2 = 0; k2 < kings.size(); ++k2) {
//             if(k1==k2) continue;
//             std::vector<Position> path1 = paths[k1];
//             std::vector<Position> path2 = paths[k2];

//             // Find the maximum path length
//             int max_length = std::max(path1.size(), path2.size());

//             // Pad the shorter path with its last position
//             while (path1.size() < max_length) {
//                 path1.push_back(path1.back());
//             }
//             while (path2.size() < max_length) {
//                 path2.push_back(path2.back());
//             }
//             for (int t = 1; t < max_length; ++t) {
                
//                 Position p1, p2;
//                 int t1 = t, t2 = t;


                
//                 if(k2>k1) {
//                     t2 = t-1;
//                 }
//                 else t2 = t;
                
//                 if(t2==-1) {
                    
//                     t2 = 0;
//                 }
//                 p1 = path1[t1];
//                 p2 = path2[t2];


//                 // Detect vertex conflict
//                 if (p1 == p2) {

//                     numberOfConflicts++;
//                     traffic[p1.x][p1.y]++;
//                     traffic[p2.x][p2.y]++;
//                 }

//                 // Detect adjacency conflict
//                 std::vector<Position> adjacents = {
//                     {p1.x + 1, p1.y}, {p1.x - 1, p1.y}, {p1.x, p1.y + 1}, {p1.x, p1.y - 1},
//                     {p1.x + 1, p1.y + 1}, {p1.x - 1, p1.y - 1}, {p1.x + 1, p1.y - 1}, {p1.x - 1, p1.y + 1}
//                 };
//                 if (std::find(adjacents.begin(), adjacents.end(), p2) != adjacents.end()) {
                    
//                     numberOfConflicts++;
//                     traffic[p1.x][p1.y]++;
//                     traffic[p2.x][p2.y]++;
//                 }
//             }
//         }
//     }
//     return numberOfConflicts;
// }

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

    using conflicts = std::pair<std::tuple<int, int, int, int>, std::tuple<int, int, int, int>>;
    std::priority_queue<ConflictPair, std::vector<ConflictPair>, decltype(cmp)> conflictQueue(cmp);
    std::vector<conflicts> conflictPairs;
    for (int k1 = 0; k1 < kings.size(); ++k1) {
        for (int k2 = 0; k2 < kings.size(); ++k2) {
            if(k1==k2) continue;
             int max_length = std::max(node.paths[k1].size(), node.paths[k2].size());

            // Pad the shorter path with its last position
            while (node.paths[k1].size() < max_length) {
                node.paths[k1].push_back(node.paths[k1].back());
            }
            while (node.paths[k2].size() < max_length) {
                if(node.paths[k2].size()==0) {
                    std::cerr<<"Empty path for king "<<k2<<"\n";
                    continue;
                }
                node.paths[k2].push_back(node.paths[k2].back());
            }

            for (int t = 1; t < max_length; ++t) {
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

                std::vector<Position> adjacents = {
                    {p1.x + 1, p1.y}, {p1.x - 1, p1.y}, {p1.x, p1.y + 1}, {p1.x, p1.y - 1},
                    {p1.x + 1, p1.y + 1}, {p1.x - 1, p1.y - 1}, {p1.x + 1, p1.y - 1}, {p1.x - 1, p1.y + 1}
                };
                if (std::find(adjacents.begin(), adjacents.end(), p2) != adjacents.end()) {
                    if(t1!=0) 
                        conflict1 = std::make_tuple(k1, t1, p1.x, p1.y);
                        
                    if(t2!=0)
                        conflict2 = std::make_tuple(k2, t2, p2.x, p2.y);
                    
                    conflictPairs.push_back({conflict1, conflict2});
                    if(t1==0 && t2==0) {
                        return false;
                    }
                    // node.traffic[k1][p1.x][p1.y]++;
                    // node.traffic[k1][p2.x][p2.y]++;
                    // node.traffic[k2][p1.x][p1.y]++;
                    // node.traffic[k2][p2.x][p2.y]++;
                    traffic[p1.x][p1.y]++;
                    traffic[p2.x][p2.y]++;
                    return true;
                }
                // Detect vertex conflict
                if (p1 == p2) {
                    if(t1!=0)
                        conflict1 = std::make_tuple(k1, t1, p1.x, p1.y);
                    if(t2!=0)
                        conflict2 = std::make_tuple(k2, t2, p2.x, p2.y);
                    if(t1==0 && t2==0) {
                        return false;
                    }
                    conflictPairs.push_back({conflict1, conflict2});
                    // node.traffic[k1][p1.x][p1.y]++;
                    // node.traffic[k1][p2.x][p2.y]++;
                    // node.traffic[k2][p1.x][p1.y]++;
                    // node.traffic[k2][p2.x][p2.y]++;
                    traffic[p1.x][p1.y]++;
                    traffic[p2.x][p2.y]++;
                    return true;
                }

                // Detect adjacency conflict
                
            }
        }
    }

    // if (!conflictQueue.empty()) {
    //     auto top_pair = conflictQueue.top();
    //     auto firstElem = top_pair.first;
    //     auto secondElem = top_pair.second;
    //     conflict1 = std::make_tuple(std::get<0>(firstElem), std::get<1>(firstElem), std::get<2>(firstElem), std::get<3>(firstElem));
    //     conflict2 = std::make_tuple(std::get<0>(secondElem), std::get<1>(secondElem), std::get<2>(secondElem), std::get<3>(secondElem));
    //     return true;
    // }


    // pick a conflict at random from the conflict pairs
    if(!conflictPairs.empty()) {
        while(1) {

            int randomIndex = rand() % conflictPairs.size();
            auto conflictPair = conflictPairs[randomIndex];
            conflict1 = std::get<0>(conflictPair);
            conflict2 = std::get<1>(conflictPair);
            // if(std::get<0>(conflict1)==-1 || std::get<0>(conflict2)==-1) {
            //     std::cerr<<"Invalid conflict pair\n";
            //     continue;
            // }
            return true;
        }
    }
    return false; // No conflicts found
}



bool ChessGame::findCardinalConflict( Node& node, const std::vector<King>& kings, std::tuple<int, int, int, int>& conflict1, std::tuple<int, int, int, int>& conflict2, int& card) {
    
    using ConflictPair = std::pair<std::tuple<int, int, int, int, int>, std::tuple<int, int, int, int, int>>; // Pair of conflicts
    // auto cmp = [this](const ConflictPair& a, const ConflictPair& b) {
    //     // Compare minimum distances to goal
    //     int a_dist_min = std::min(std::get<4>(a.first), std::get<4>(a.second));
    //     int b_dist_min = std::min(std::get<4>(b.first), std::get<4>(b.second));
    //     return a_dist_min > b_dist_min; // Greater distance should be placed at the end
    // };
    // std::priority_queue<ConflictPair, std::vector<ConflictPair>, decltype(cmp)> conflictQueue(cmp);
    std::vector<ConflictPair> conflictPairs;
    
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

            for (int t = 1; t < max_length; ++t) {
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
                    if(t1!=0)
                        conflict1 = std::make_tuple(k1, t1, p1.x, p1.y);
                    if(t2!=0)
                        conflict2 = std::make_tuple(k2, t2, p2.x, p2.y);
                    if(t1==0 && t2==0) {
                        return false;
                    }
                    // traffic[p1.x][p1.y]++;
                    // traffic[p2.x][p2.y]++;
                    // return true;
      
                    // conflictQueue.emplace(k1, t, p1.x, p1.y, min_dist_to_goal);
                    auto conflictPair = std::make_pair(std::make_tuple(k1, t, p1.x, p1.y, min_dist_to_goal), std::make_tuple(k2, t2, p2.x, p2.y, min_dist_to_goal));
                    conflictPairs.push_back(conflictPair);
                }

                // Detect adjacency conflict
                std::vector<Position> adjacents = {
                    {p1.x + 1, p1.y}, {p1.x - 1, p1.y}, {p1.x, p1.y + 1}, {p1.x, p1.y - 1},
                    {p1.x + 1, p1.y + 1}, {p1.x - 1, p1.y - 1}, {p1.x + 1, p1.y - 1}, {p1.x - 1, p1.y + 1}
                };
                if (std::find(adjacents.begin(), adjacents.end(), p2) != adjacents.end()) {
                    if(t1!=0)
                        conflict1 = std::make_tuple(k1, t1, p1.x, p1.y);
                    if(t2!=0)
                        conflict2 = std::make_tuple(k2, t2, p2.x, p2.y);
                     if(t1==0 && t2==0) {
                        return false;
                    }
                    // traffic[p1.x][p1.y]++;
                    // traffic[p2.x][p2.y]++;
                    // return true;
                    auto conflictPair = std::make_pair(std::make_tuple(k1, t, p1.x, p1.y, min_dist_to_goal), std::make_tuple(k2, t2, p2.x, p2.y, min_dist_to_goal));
                    conflictPairs.push_back(conflictPair);
                }
            }
        }
    }

    if(conflictPairs.empty()) {
        return false;
    }
    
    // for every conflict pair in the queue, check if it is cardinal conflict
    // for every conflict, call astar with a constraint and calculate the cost
    // see if it both costs are greater than the current cost
    // if yes, return true
    // else return false
    for(auto conflictPair: conflictPairs){
        
        auto firstElem = conflictPair.first;
        auto secondElem = conflictPair.second;
        int k1 = std::get<0>(firstElem);
        int k2 = std::get<0>(secondElem);
        conflict1 = std::make_tuple(k1, std::get<1>(firstElem), std::get<2>(firstElem), std::get<3>(firstElem));
        conflict2 = std::make_tuple(k2, std::get<1>(secondElem), std::get<2>(secondElem), std::get<3>(secondElem));
        // add the constraint to the node
        // copy existing node 
        Node copyNode = node;
        copyNode.constraints[k1].insert({std::get<1>(firstElem), std::get<2>(firstElem), std::get<3>(firstElem)});
        copyNode.constraints[k2].insert({std::get<1>(secondElem), std::get<2>(secondElem), std::get<3>(secondElem)});
        int startTimeStep1 = 0;
        // call astar for each and check cost
        std::vector<Position> newPath1 = lowLevelSearch(k1, 0, copyNode, startTimeStep1);
        //if not exmpty, add king's start position as first element to new path and update the path
        auto paths = copyNode.paths;
        int existingCost = copyNode.paths[k1].size();
        if (!newPath1.empty()) {
            int pathSize = paths[k1].size();
            paths[k1].clear();
            paths[k1].push_back(kings[k1].start);
            for(int i=1;i<=newPath1.size();i++) {
                paths[k1].push_back(newPath1[i-1]);
            }
            if(paths[k1].size()<pathSize) {
                while(paths[k1].size()<pathSize) {
                    paths[k1].push_back(paths[k1].back());
                }
            }
        }
        else {
            std::cerr<<"Empty path for king "<<k1<<"\n";
        }

        
        card = -1;
        int newcost_1 = paths[k1].size();
        if(newcost_1>existingCost) {
            card = 0;
        }
        
        int startTimeStep2 = 0;
        std::vector<Position> newPath2 = lowLevelSearch(k2, 0, copyNode, startTimeStep2);
        if (!newPath2.empty()) {
            int pathSize = paths[k2].size();
            paths[k2].clear();
            paths[k2].push_back(kings[k2].start);
            for(int i=1;i<=newPath2.size();i++) {
                paths[k2].push_back(newPath2[i-1]);
            }
            if(paths[k2].size()<pathSize) {
                while(paths[k2].size()<pathSize) {
                    paths[k2].push_back(paths[k2].back());
                }
            }
        }
        else {
            std::cerr<<"Empty path for king "<<k2<<"\n";
        }

        int newcost_2 = paths[k2].size();
        if(newcost_2>existingCost) {
            if(card==0) {
                card = 1;
                return true;
            }
            else {
                card = 0;
            }
        }
    }


    return true; // No conflicts found
}

bool ChessGame::checkFutureConstraints(const Position toCheck, const int currTimeStep, const Node curr, const int kingIndex) {
    // get the maximum path size among all paths
    // start from curr timestep, toCheckPosition and go till max value
    // check for constraints 
    // return true if constraints are violated
    // else return false
    int maxPathSize = 0;
    for(int i=0;i<kings.size();i++) {
        if(curr.paths[i].size()>maxPathSize) {
            maxPathSize = curr.paths[i].size();
        }
    }
    const auto& king_constraints = curr.constraints[kingIndex];
            
    for(int t=currTimeStep+1;t<=maxPathSize;t++) {
        Constraint c{t, toCheck.x, toCheck.y};
        if (king_constraints.find(c) != king_constraints.end()) {
            return true;
        }
    }
    return false;

}

std::size_t ChessGame::generateTrafficSignature(const std::vector<std::vector<int>>& traffic) {
    std::size_t hash = 0;
    for (int i = 0; i < traffic.size(); i += 1) {
        for (int j = 0; j < traffic[i].size(); j += 1) {
            hash ^= std::hash<int>{}(traffic[i][j]) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
    }
    return hash;
}

std::vector<Position> ChessGame::lowLevelSearch(const int kingIndex, int startTime, const Node curr, int& startTimeStep) {
    // std::cout<<"Low level search for king: "<<kingIndex<<"\n";
    std::priority_queue<std::tuple<int, int, Position>, std::vector<std::tuple<int, int, Position>>, std::greater<>> open_list;
    std::unordered_map<std::tuple<Position, int>, int, PositionTimeHasher> cost_map;
    std::unordered_map<std::tuple<Position, int>, std::tuple<Position, int>, PositionTimeHasher> came_from;
    std::unordered_set<std::tuple<Position, int>, PositionTimeHasher> closed_list;

    Position start = kings[kingIndex].start;
    std::unordered_set<Constraint, ConstraintHasher> king_constraints = curr.constraints[kingIndex];
    Position target = kings[kingIndex].target;
    // auto trafficSignature = generateTrafficSignature(traffic);
    auto cacheKey = std::make_tuple(start, target, king_constraints);
    bool congested = false;
    if(size<=8 && kings.size()==size) {
        congested = true;
    }

    // if(!congested) {
    //     if (pathCache.find(cacheKey) != pathCache.end()) {
    //         stats.cacheHits++;
    //         return pathCache[cacheKey];
    //     }
    // }

    std::tuple<Position, int> start_key = {start, startTime};
    open_list.push({0, startTime, start});
    cost_map[{start, startTime}] = 0;

    while (!open_list.empty()) {
        auto [cost, timeStep, current] = open_list.top();
        open_list.pop();
        std::tuple<Position, int> current_key = {current, timeStep};
        closed_list.insert(current_key);
        if (current == kings[kingIndex].target && !checkFutureConstraints(current, timeStep-1, curr, kingIndex)) {
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
            pathCache[cacheKey] = path; 
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
            std::tuple<Position, int> next_key = {next, timeStep + 1};
            

            

            bool conflict = false;

            Constraint c{timeStep+1, next.x, next.y};
            if (king_constraints.find(c) != king_constraints.end()) {
                conflict = true;
            }

            
            if (conflict) continue;
            if (closed_list.find(next_key) != closed_list.end()) continue;
            int weight = 1;

            int heuristic_cost = weight*kings[kingIndex].distance_map[next]  + traffic[next.x][next.y];
            // + curr.traffic.at(kingIndex)[next.x][next.y]
            //  traffic[next.x][next.y]
            // int heuristic_cost = manhattanDistance(next, kings[kingIndex].target);
            int new_cost = cost + 1 + heuristic_cost ;

            if (cost_map.find(next_key) == cost_map.end() || new_cost < cost_map[next_key]) {
                cost_map[next_key] = new_cost;
                came_from[next_key] = {current, timeStep};
                open_list.push({new_cost, timeStep + 1, next});
            }
        }
    }
    return {}; // No valid path found
}

// int calculateCost(const MetaAgent::MetaState& state) {
//     int totalCost = 0;
//     for (const auto& pos : state.positions) {
//         totalCost += heuristic(pos);
//     }
//     return totalCost;
// }



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