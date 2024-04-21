#include <CBSAlgorithm.h>

CBSAlgorithm::CBSAlgorithm(const std::vector<King>& kings, const std::vector<std::vector<bool>>& board, int size)
    : kings(kings), board(board), boardSize(size), solutionPaths() {}

bool CBSAlgorithm::findPaths() {
        Node root;
        root.paths.resize(kings.size());

        if (!findPathsForNode(root)) {
            std::cout << " solution not found at root level." << std::endl;
            return false;
        }

        std::priority_queue<Node, std::vector<Node>, CompareNode> open;
        open.push(root);

        while (!open.empty()) {
            Node current = open.top();
            open.pop();

            Conflict conflict = detectConflict(current);
            if (conflict.king1 == -1) {
                std::cout << "Solution found!" << std::endl;
                outputSolution(current);
                return true;
            }

            resolveConflict(current, conflict, open);
        }

        std::cout << "No solution exists." << std::endl;
        return false;
}

bool CBSAlgorithm::findPathsForNode(Node& node) {
    node.paths.resize(kings.size());
    for (int i = 0; i < kings.size(); ++i) {
        const auto& constraints = node.constraints[{i, i}];  // Get constraints for this king
        node.paths[i] = AStar::search(board, kings[i].start, kings[i].goal, constraints);
        if (node.paths[i].empty()) return false;  // No path found for this king
    }
    return true;
}

Conflict CBSAlgorithm::detectConflict(const Node& node) {
    std::map<Position, int> positions;
    for (int i = 0; i < node.paths.size(); ++i) {
        for (const auto& pos : node.paths[i]) {
            if (positions.count(pos)) {
                return {positions[pos], i, pos.time, pos};  // Conflict found
            }
            positions[pos] = i;
        }
    }
    return Conflict();  // No conflict found
}

void CBSAlgorithm::resolveConflict(Node& node, const Conflict& conflict, std::priority_queue<Node, std::vector<Node>, CompareNode>& open) {
    Node child1 = node, child2 = node;

    // Add constraints to each child to avoid the conflicting position at the conflicting time
    child1.constraints[{conflict.king1, conflict.king1}].insert(conflict.pos);
    child2.constraints[{conflict.king2, conflict.king2}].insert(conflict.pos);

    if (findPathsForNode(child1)) open.push(child1);
    if (findPathsForNode(child2)) open.push(child2);
}

void CBSAlgorithm::outputSolution(const Node& node) {
    solutionPaths = node.paths;
    for (int i = 0; i < solutionPaths.size(); ++i) {
        std::cout << "King " << (i + 1) << " path: ";
        for (const auto& pos : solutionPaths[i]) {
            std::cout << "(" << pos.x << ", " << pos.y << ") ";
        }
        std::cout << std::endl;
    }
}