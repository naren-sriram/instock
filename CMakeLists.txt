# Set the minimum required version of cmake for this project
cmake_minimum_required(VERSION 3.10)

# Set project name
project(CBS)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories where header files are located
include_directories(include)

# Add source files to the executable
# add_executable(mapp src/main.cpp src/PathFinder.cpp src/GameSimulator.cpp)
# add_executable(SIPP src/main.cpp src/SIPP.cpp )
add_executable(ChessGame src/main.cpp src/ChessGame.cpp )
# Optional: Define macros or compiler options
# add_definitions(-DMY_MACRO=1)
# target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
