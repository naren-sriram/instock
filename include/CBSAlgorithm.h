// CBS.h
#ifndef CBS_H
#define CBS_H

#include <iostream>
#include <queue>
#include <limits>
#include <AStar.h>


class CBSAlgorithm {
public:
    std::vector<King> kings;
    std::vector<std::vector<bool>> board;
    int boardSize;
    std::vector<std::vector<Position>> solutionPaths; 
    CBSAlgorithm(const std::vector<King>& kings, const std::vector<std::vector<bool>>& board, int size);


    bool findPaths();

private:
    bool findPathsForNode(Node& node);

    Conflict detectConflict(const Node& node);

    void resolveConflict(Node& node, const Conflict& conflict, std::priority_queue<Node, std::vector<Node>, CompareNode>& open);

    void outputSolution(const Node& node);
};

#endif // CBS_H