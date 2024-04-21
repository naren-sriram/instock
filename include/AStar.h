#ifndef ASTAR_H
#define ASTAR_H
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <cmath>  // For std::abs
#include <algorithm>
#include<Utility.h>
#include <iostream>
// Additional utility functions
inline int manhattanDistance(const Position& a, const Position& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

struct Vertex {
    Position position;
    int priority;
    int cost;
    Vertex(Position pos, int pri, int c) : position(pos), priority(pri), cost(c) {}
    bool operator>(const Vertex& other) const {
        return priority > other.priority; // Min-Heap
    }
};
class AStar {
public:
   static std::vector<Position> search(const std::vector<std::vector<bool>>& board,
                                  const Position& start,
                                  const Position& goal,
                                  const std::set<Position>& constraints) ;
};
#endif
