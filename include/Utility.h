#ifndef UTILITY_H
#define UTILITY_H
#include <vector>
#include <map>
#include <set>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <queue>
#include<unordered_set>
struct Position {
    int x, y;
    Position(int x = 0, int y = 0) : x(x), y(y) {}

    // Equality operator
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }

    // Less-than operator to use Position as a key in ordered containers
    bool operator<(const Position& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }
};

// Hash function for Position to use in unordered containers
struct PositionHasher {
    std::size_t operator()(const Position& pos) const {
        // XOR-based combination of hash values of x and y
        return std::hash<int>()(pos.x) ^ std::hash<int>()(pos.y) ;
    }
};

struct King {
    Position current, target, start;
    std::vector<Position> path;
    std::unordered_map<Position, int, PositionHasher> distance_map;
    int waitCount = 0;
    
    King(Position start, Position end) : start(start), current(start), target(end) {}
};

struct Constraint {
    int time;
    int x, y;
    bool operator==(const Constraint& other) const {
        return time == other.time && x == other.x && y == other.y;
    }
};

// Custom hash function for Constraint
struct ConstraintHasher {
    std::size_t operator()(const Constraint& c) const {
        return std::hash<int>()(c.time) ^ std::hash<int>()(c.x) ^ std::hash<int>()(c.y);
    }
};


struct PositionTimeHasher {
    std::size_t operator()(const std::tuple<Position, int>& p) const {
        std::size_t pos_hash = PositionHasher{}(std::get<0>(p));
        std::size_t time_hash = std::hash<int>{}(std::get<1>(p));
        return hashCombine(pos_hash, time_hash); // Combine using hashCombine
    }

    std::size_t hashCombine(std::size_t lhs, std::size_t rhs) const {
        lhs ^= rhs + 0x9e3779b9 + (lhs << 6) + (lhs >> 2);
        return lhs;
    }
};

struct MetaConstraint {
    std::unordered_set<int> agents; // Set of agent indices involved in the meta-agent
    Position position;
    int time;

    MetaConstraint(std::unordered_set<int> agents, Position position, int time)
        : agents(std::move(agents)), position(position), time(time) {}

    bool operator==(const MetaConstraint& other) const {
        return agents == other.agents && position == other.position && time == other.time;
    }
};

// Hash function for MetaConstraint to use in unordered containers
struct MetaConstraintHasher {
    std::size_t operator()(const MetaConstraint& mc) const {
        std::size_t seed = 0;
        for (const auto& agent : mc.agents) {
            seed ^= std::hash<int>()(agent);
        }
        return seed ^ std::hash<int>()(mc.position.x) ^ std::hash<int>()(mc.position.y) ^ std::hash<int>()(mc.time);
    }
};

struct Node {
    std::vector<std::vector<Position>> paths;
    mutable std::unordered_map<int, std::unordered_set<Constraint, ConstraintHasher>> constraints; // Set of constraints per king
    std::unordered_set<MetaConstraint, MetaConstraintHasher> metaConstraints;
    std::unordered_map<int,int> numConflicts;
    int totalConflicts = 0;
    int cost;
    // std::unordered_map<int, std::vector<std::vector<int>>> traffic;
    Node(int k) : paths(k), cost(0) {}
};

struct TrafficAwarePathCacheHasher {
    std::size_t operator()(const std::tuple<Position, Position, std::size_t>& key) const {
        const auto& [start, target, trafficHash] = key;
        std::size_t hash = PositionHasher{}(start) ^ PositionHasher{}(target);
        return hash ^ trafficHash;
    }
};






struct CompareNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.cost > b.cost; // This will make the priority queue a min-heap based on cost
    }
};

struct PathCacheHasher {
    // Hash a single Position object
    std::size_t hashPosition(const Position& pos) const {
        return std::hash<int>{}(pos.x) ^ std::hash<int>{}(pos.y);
    }

    // Hash a set of constraints
    std::size_t hashConstraints(const std::unordered_set<Constraint, ConstraintHasher>& constraints) const {
        std::size_t hash = 0;
        for (const auto& constraint : constraints) {
            hash ^= ConstraintHasher{}(constraint) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
        return hash;
    }

    std::size_t operator()(const std::tuple<Position, Position, std::unordered_set<Constraint, ConstraintHasher>>& key) const {
        const auto& [start, target, constraints] = key;

        // Combine hashes of the start position, target position, and constraints
        std::size_t hash = hashPosition(start);
        hash ^= hashPosition(target) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= hashConstraints(constraints) + 0x9e3779b9 + (hash << 6) + (hash >> 2);

        return hash;
    }
};


struct Statistics {
        int expandedNodes = 0;
        int generatedNodes = 0;
        int cost = 0;
        int iterations = 0;
        int cacheHits = 0;
};


struct ConflictHasher {
    std::size_t operator()(const std::pair<std::tuple<int, int, int, int>, std::tuple<int, int, int, int>>& c) const {
        auto hasher = std::hash<int>();
        return hasher(std::get<0>(c.first)) ^ hasher(std::get<1>(c.first)) ^ hasher(std::get<2>(c.first)) ^ hasher(std::get<3>(c.first))
             ^ hasher(std::get<0>(c.second)) ^ hasher(std::get<1>(c.second)) ^ hasher(std::get<2>(c.second)) ^ hasher(std::get<3>(c.second));
    }
};


struct Conflict {
    int king1, king2, time;
    Position pos;
    Conflict(int k1 = -1, int k2 = -1, int time = -1, Position p = Position()) : king1(k1), king2(k2), time(time), pos(p) {}
};







static std::vector<std::vector<bool>> readBoard(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        throw std::runtime_error("Failed to open the board file: " + filename);
    }

    int size;
    if (!(file >> size)) {
        throw std::runtime_error("Failed to read the size of the board from the file: " + filename);
    }

    std::vector<std::vector<bool>> board(size, std::vector<bool>(size, false)); // False means open, true means blocked
    std::string line;
    std::getline(file, line); // Skip the rest of the line after reading size

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int x, y;
        char comma;
        if (!(iss >> x >> comma >> y) || comma != ',' || x >= size || y >= size || x < 0 || y < 0) {
            throw std::runtime_error("Invalid or out-of-bounds board position in file: " + filename);
        }
        board[x][y] = true; // Mark the position as blocked
    }

    return board;
}





static std::vector<King> readKings(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        throw std::runtime_error("Failed to open the kings file: " + filename);
    }

    std::vector<King> kings;
    std::string line;
    while (getline(file, line)) {  // Read each line from the file
        std::istringstream ss(line);
        std::vector<int> positions;
        std::string number;
        while (getline(ss, number, ',')) {  // Split the line by commas
            try {
                // Convert the read string to integer and add to positions
                positions.push_back(std::stoi(number));
            } catch (const std::invalid_argument& e) {
                throw std::runtime_error("Invalid number in kings file: " + number);
            } catch (const std::out_of_range& e) {
                throw std::runtime_error("Number out of range in kings file: " + number);
            }
        }

        if (positions.size() != 4) {  // Ensure exactly four integers per king
            throw std::runtime_error("Incorrect number of coordinates for a king in the file: " + line);
        }

        // Create a King object and add it to the list of kings
        kings.emplace_back(Position(positions[0], positions[1]), Position(positions[2], positions[3]));
    }

    if (!file.eof()) {
        throw std::runtime_error("Error reading kings data from file: " + filename);
    }

    return kings;
}

#endif