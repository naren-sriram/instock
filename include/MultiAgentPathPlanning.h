// Assume King, Position, and other structs are already defined and included
#include "Utility.h"  // Contains King, Position, Node, etc.
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <unordered_map>

class MultiAgentPathPlanning {
public:
    MultiAgentPathPlanning(const std::vector<King>& kings, const std::vector<std::vector<bool>>& board, int size);
    bool findPaths();
    void outputSolution();
    std::vector<std::vector<Position>> allPaths; // To store all paths
private:
    std::vector<King> kings;
    std::vector<std::vector<bool>> board;
    int boardSize;
    

    bool planPathForKing(int kingIndex, int timeOffset);
    std::vector<Position> getNextPossiblePositions(const Position& current, const std::set<Position>& occupied);
    int heuristic(const Position& a, const Position& b);
    bool isBlocked(const Position& pos);
    bool isOccupiedByOtherKings(const Position& pos, int kingIndex, int timeStep) ;
    void rebuildPath(int kingIndex, const Position& start, const Position& goal, const std::map<Position, Position>& cameFrom);
    std::vector<Position> getNeighbors(const Position& pos, int timeOffset);
    void updateKingPaths(int updatedKingIndex);
    bool isOccupiedByFutureKingPositions(const Position& pos, int kingIndex, int timeOffset);
    std::vector<int> validatePaths();
    bool replanPath(int kingIndex);
};

