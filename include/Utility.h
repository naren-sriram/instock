#ifndef UTILITY_H
#define UTILITY_H
#include <vector>
#include <map>
#include <set>
#include <iostream>
#include <fstream>
#include <sstream>
struct Position {
    int x, y, time;
    Position(int x = 0, int y = 0, int time = 0) : x(x), y(y), time(time) {}
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y && time == other.time;
    }
    bool operator<(const Position& other) const {
        return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
    }

};

struct King {
    Position start;
    Position goal;
    King(Position s, Position g) : start(s), goal(g) {}
};

struct Node {
    std::vector<std::vector<Position>> paths;  // paths for each king
    std::map<std::pair<int, int>, std::set<Position>> constraints;  // conflicts resolved per king
    int cost;
    Node() : cost(0) {}

    Node(const Node& other) : paths(other.paths), constraints(other.constraints), cost(other.cost) {}
};

struct CompareNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.cost > b.cost; // This will make the priority queue a min-heap based on cost
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