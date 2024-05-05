// ChessGame.h
#ifndef CHESSGAME_H
#define CHESSGAME_H

#include <Utility.h>
#include <unordered_set>
#include <unordered_map>
#include <optional>
#include <algorithm>
#include <queue>
#include <MetaAgent.h>

class ChessGame {

public:
    ChessGame(int n, const std::vector<std::vector<bool>>& initialGrid, std::vector<King>& allKings);
    bool findPathsCBS();
    void writePathsToFile(const std::string& filename);
    Statistics stats;


private:

    std::unordered_map<std::tuple<Position, Position, std::unordered_set<Constraint, ConstraintHasher>>, std::vector<Position>, PathCacheHasher> pathCache;
    int size;
    std::vector<std::vector<int>> grid; // Changed from bool to int
    std::vector<King> kings;
    std::size_t generateTrafficSignature(const std::vector<std::vector<int>>& traffic) ;
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
    
    std::vector<Position> lowLevelSearch(const int kingIndex, int startTime, const Node curr, int& startTimeStep);
    bool findConflicts(  Node& node,  const std::vector<King>& kings, std::tuple<int, int, int, int>& conflict1, std::tuple<int, int, int, int>& conflict2);
    std::unordered_map<Position, int, PositionHasher> calculateDijkstraMap(const Position& goal, const std::vector<std::vector<int>>& grid);
    int calculateCost(const std::vector<std::vector<Position>>& paths);
    int manhattanDistance(const Position& a, const Position& b);
    int updateTraffic(const std::vector<std::vector<Position>>& paths);
    bool isValidKingMove(const Position& current, const Position& next) ;
    bool isPathValid(const std::vector<Position>& path) ;
    std::vector<std::vector<int>> traffic;
    bool checkFutureConstraints(const Position toCheck, const int currTimeStep, const Node curr, const int kingIndex);
    bool findCardinalConflict( Node& node, const std::vector<King>& kings, std::tuple<int, int, int, int>& conflict1, std::tuple<int, int, int, int>& conflict2, int& cardinality);
    std::vector<Position> lowLevelMetaAgentSearch(std::vector<int> metaAgent, int startTime, const Node curr, int& startTimeStep) ;
    int calculateCost(const MetaAgent::MetaState& state);
    int calculateNumberOfConflictsPerAgent( Node& node, const std::vector<std::vector<Position>>& paths, int k1);
    void initializeTrafficTable(Node& node, int rows, int cols);
};

#endif // CHESSGAME_H
