// ChessGame.h
#ifndef CHESSGAME_H
#define CHESSGAME_H

#include <Utility.h>


class ChessGame {

public:
    ChessGame(int n, const std::vector<std::vector<bool>>& initialGrid);
    void placeKings( std::vector<King>& allKings);
    bool findPaths();
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
};

#endif // CHESSGAME_H
