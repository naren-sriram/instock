// Main.cpp

#include <vector>
#include <CBSAlgorithm.h>
#include <chrono>

int main() {
    
    std::string mapFile = "/home/naren/instock/problem-tests/3/map.txt";
    std::string kingsFile = "/home/naren/instock/problem-tests/3/kings.txt";
    std::vector<std::vector<bool>> board = readBoard(mapFile);
    std::vector<King> kings = readKings(kingsFile);

    std::cout<<"kings size: "<<kings.size()<<"\n";


    // Instantiate CBS algorithm and run
    CBSAlgorithm cbs(kings, board, board.size());
    auto start = std::chrono::high_resolution_clock::now(); 
    if(cbs.findPaths()) {
        std::cout<<"Exited cbs \n";
    }
    else {
        std::cout<<"Could not find solution \n";
        return 0;
    }
    auto end = std::chrono::high_resolution_clock::now();    // End timing
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Planning time: " << elapsed.count() << " seconds." << std::endl;

    std::ofstream trajFile("trajectory_3.txt");
    for (size_t i = 0; i < kings.size(); ++i) {
        trajFile << "King " << i + 1 << " path: ";
        for (const auto& pos : cbs.solutionPaths[i]) {
            trajFile << "(" << pos.x << "," << pos.y << ") ";
        }
        trajFile << std::endl;
    }
    trajFile<< "Planning time: "<<elapsed.count()<<" seconds."<<std::endl;
    trajFile.close();

    return 0;
}
