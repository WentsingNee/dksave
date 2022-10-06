cmake_minimum_required(VERSION 3.23)
project(DKSave CXX)

set(CMAKE_CXX_STANDARD 20)

add_executable(dksave src/dksave.cpp)
target_include_directories(dksave PUBLIC include)
