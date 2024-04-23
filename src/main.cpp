#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <ChessGame.h> // Include the new GameSimulator class

int main() {
    std::string mapFile = "/home/naren/instock/problem-tests/1/map.txt";  // Adjusted path for simplicity
    std::string kingsFile = "/home/naren/instock/problem-tests/1/kings.txt";  // Adjusted path for simplicity
    std::vector<std::vector<bool>> board = readBoard(mapFile);
    std::vector<King> kings = readKings(kingsFile);
    std::cout<<"read operations completed. \n";
    ChessGame ChessGame(board.size(), board);
    ChessGame.placeKings(kings);
    auto start = std::chrono::high_resolution_clock::now();
    ChessGame.findPaths();
        
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::string solution_path = "solution_1.txt";
    ChessGame.writePathsToFile(solution_path);

    std::ofstream file(solution_path, std::ios_base::app | std::ios_base::out);
    if (file.is_open()) {
        file << "Execution Time: " << duration.count() << " ms" << std::endl;
        file.close();
    } else {
        std::cerr << "Failed to open file for appending execution time.\n";
    }

    std::cout << "Pathfinding complete. Results written to "<<solution_path << std::endl;
    return 0;
}
