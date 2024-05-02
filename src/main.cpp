#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <map>
#include <string>
#include <ChessGame.h>  // Include the GameSimulator class

int main(int argc, char* argv[]) {
    std::map<std::string, std::string> cmdArgs;
    for (int i = 1; i < argc; i += 2) {
        if (i + 1 < argc) { // Make sure we aren't at the end of argv!
            cmdArgs[argv[i]] = argv[i + 1];  // Insert into map
        } else {
            std::cerr << "Missing value for " << argv[i] << std::endl;
            return 1; // Missing argument value
        }
    }

    std::string mapFile = cmdArgs.count("input_map") ? cmdArgs["input_map"] : "problem-tests/8/map.txt";  // Get the map file path
    std::string kingsFile = cmdArgs.count("input_kings") ? cmdArgs["input_kings"] : "problem-tests/8/kings.txt";  // Get the kings file path
    std::string solutionPath = cmdArgs.count("solution") ? cmdArgs["solution"] : "solution_8.txt";  // Get the solution file path or default

    if (mapFile.empty() || kingsFile.empty()) {
        std::cerr << "Usage: " << argv[0] << " input_map <map_file> input_kings <kings_file> solution <output_file>s" << std::endl;
        return 1;
    }

    std::vector<std::vector<bool>> board = readBoard(mapFile);
    std::vector<King> kings = readKings(kingsFile);
    ChessGame chessGame(board.size(), board, kings);
    // chessGame.placeKings(kings);

    auto start = std::chrono::high_resolution_clock::now();
    bool found = chessGame.findPathsCBS();
    auto stop = std::chrono::high_resolution_clock::now();

    std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    chessGame.writePathsToFile(solutionPath);

    std::ofstream file(solutionPath, std::ios::app);
    file.close();

    std::cout << "Pathfinding complete. Results written to " << solutionPath << "Execution Time: " << duration.count() << " ms"<<std::endl;
    return 0;
}
