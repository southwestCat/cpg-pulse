cmake_minimum_required(VERSION 3.20)

project(cpg)

include_directories(Src)

set(SRCs
    Src/Correlation.cpp
    Src/Fitness.cpp
    Src/Rk4.cpp
    Src/PSO.cpp
    Src/Matsuoka.cpp
    Src/Queue.cpp
)

option(DEBUG_OPTION "debug check memory leak." OFF)

if(DEBUG_OPTION)
    message(STATUS "Debug Mode. Run program and check memory leak.")
    add_definitions("-Wall -g -pipe -fsanitize=address -std=c++17")

    add_executable(${PROJECT_NAME} Src/Main.cpp ${SRCs})
    target_link_libraries(${PROJECT_NAME} -pthread -lfftw3 -lasan)
    
    add_executable(test test.cpp ${SRCs})
    target_link_libraries(test -pthread -lfftw3 -lasan)
    
    add_executable(corr corr.cpp)
    target_link_libraries(corr -lfftw3 -lasan)
else()
    message(STATUS "Release Mode...")
    add_definitions("-Wall -O3 -pipe -std=c++20")

    add_executable(${PROJECT_NAME} Src/Main.cpp ${SRCs})
    target_link_libraries(${PROJECT_NAME} -pthread -lfftw3)
    
    add_executable(test test.cpp ${SRCs})
    target_link_libraries(test -pthread -lfftw3)
    
    add_executable(bench bench.cpp ${SRCs})
    target_link_libraries(bench -lfftw3 -pthread)

    add_executable(corr corr.cpp)
    target_link_libraries(corr -lfftw3)
endif()
