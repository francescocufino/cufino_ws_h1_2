cmake_minimum_required(VERSION 3.5)
project(h1_2_manipulation VERSION 2.0.0)


## Project Options

## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

add_subdirectory(src/h1_2_motion)
add_subdirectory(src/h1_2_demo)
add_subdirectory(src/h1_2_pushing)
