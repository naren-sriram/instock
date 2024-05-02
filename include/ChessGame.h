// ChessGame.h
#ifndef CHESSGAME_H
#define CHESSGAME_H

#include <Utility.h>
#include <unordered_set>
#include <unordered_map>
#include <optional>
#include <algorithm>
#include <queue>
class ChessGame {

public:
    ChessGame(int n, const std::vector<std::vector<bool>>& initialGrid, std::vector<King>& allKings);
    bool findPathsCBS();
    void writePathsToFile(const std::string& filename);

private:
    int size;
    std::vector<std::vector<int>> grid; // Changed from bool to int
    std::vector<King> kings;

    bool noSolution = false;
    void updateAdjacentCells(Position pos, bool increment);
    bool isFree(const Position& pos) const;
    int heuristic(const Position& current, int kingIndex) const;   
    std::vector<Position> getNeighbors(const Position& pos) const ;
    Position findReevaluationMove(const King& king, int kingIndex, bool isWaiting);
    void computeDistanceMaps();
    std::vector<std::vector<int>> computeDijkstraMap(const Position& goal) ;
    std::vector<std::vector<std::vector<int>>> distanceMaps;
    void unblock(int kingIndex);
    void moveBlockingKing(int kingIndex, int blockingKingIndex);
    int identifyBlockingKing(int kingIndex);
    std::unordered_map<int,int> entangled;
    bool withinBounds(const Position& pos);
    
    std::vector<Position> lowLevelSearch(int kingIndex, int startTime, Node curr);
    bool findConflicts(const Node& node, const std::vector<King>& kings, std::tuple<int, int, int, int>& conflict1, std::tuple<int, int, int, int>& conflict2);
    std::unordered_map<Position, int, PositionHasher> calculateDijkstraMap(const Position& goal, const std::vector<std::vector<int>>& grid);
    int calculateCost(const std::vector<std::vector<Position>>& paths);
    int manhattanDistance(const Position& a, const Position& b);
};

#endif // CHESSGAME_H
