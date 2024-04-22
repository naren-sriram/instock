#include <iostream>
#include <fstream>
#include <vector>
#include <MultiAgentPathPlanning.h> 
#include <chrono>

int main() {
    std::string mapFile = "/home/naren/instock/problem-tests/1/map.txt";
    std::string kingsFile = "/home/naren/instock/problem-tests/1/kings.txt";
    std::vector<std::vector<bool>> board = readBoard(mapFile);
    std::vector<King> kings = readKings(kingsFile);

    MultiAgentPathPlanning planner(kings, board, board.size());
    auto start = std::chrono::steady_clock::now();

    if (!planner.findPaths()) {
        std::cout << "Failed to find paths for all kings.\n";
        return 1;
    }

    auto end = std::chrono::steady_clock::now();
    std::cout<<"PLannign successful! \n";
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Planning time: " << elapsed.count() << " seconds.\n";

    std::ofstream solutionFile("solution_test.txt");
    if (!solutionFile.is_open()) {
        std::cerr << "Unable to open solution file.\n";
        return 1;
    }

    int maxSteps = 0;
    for (const auto& path : planner.allPaths) {
        maxSteps = std::max(maxSteps, static_cast<int>(path.size()));
    }

    // Iterate over time steps, for each step iterate over all kings
    for (int step = 0; step < maxSteps; ++step) {
        for (size_t kingIndex = 0; kingIndex < planner.allPaths.size(); ++kingIndex) {
            if (step < planner.allPaths[kingIndex].size()) {
                const auto& pos = planner.allPaths[kingIndex][step];
                solutionFile << pos.x << ", " << pos.y << std::endl;
            } else if (!planner.allPaths[kingIndex].empty()) {
                // If no more steps, repeat the last position
                const auto& lastPos = planner.allPaths[kingIndex].back();
                solutionFile << lastPos.x << ", " << lastPos.y << std::endl;
            }
        }
    }

    solutionFile << "Planning time: " << elapsed.count() << " seconds." << std::endl;
    solutionFile.close();

    return 0;
}
