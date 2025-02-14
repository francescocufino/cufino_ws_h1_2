cmake_minimum_required(VERSION 3.5)
project(h1_2+manipulation VERSION 2.0.0)

## Project Options
option(BUILD_H1_2_DEMO "Build h1_2_demo" ON)

## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

add_subdirectory(h1_2_motion)

if (BUILD_H1_2_DEMO)
	add_subdirectory(h1_2_demo)
endif ()

