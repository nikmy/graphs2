cmake_minimum_required(VERSION 3.16)
project(Graphs_2)

set(CMAKE_CXX_STANDARD 17)

set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -fsanitize=address -fsanitize=undefined")
set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -Wall -Wextra -fsanitize=address -fsanitize=undefined")

add_executable(${PROJECT_NAME} N_Floyd_Warshall.cpp)
