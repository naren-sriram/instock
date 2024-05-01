#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <set>
#include <queue>




struct Position {
    int x, y;
    Position(int x = 0, int y = 0) : x(x), y(y) {}
    bool operator==(const Position& other) const { return x == other.x && y == other.y; }
    bool operator<(const Position& other) const { return x < other.x || (x == other.x && y < other.y); }
};

struct HeuristicNode {
    Position pos;
    int heuristic;
    bool operator>(const HeuristicNode& other) const {
        return heuristic > other.heuristic;
    }
};


struct Node {
    Position pos;
    int heuristic;  // Heuristic cost to goal
    bool operator>(const Node& other) const { return heuristic > other.heuristic; }
};

struct King {
    Position start;
    Position goal;
    Position current;
    Position previous;
    std::vector<Position> path;
    std::set<Position> closedList;
    int waitCount = 0;
    King(Position s, Position g) : start(s), goal(g) {
        current = s;
    }
    bool unblockedOnce = false;
    int crossedLongWait = 0;
    bool blocked = false;
    bool cannotPlan = false;
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