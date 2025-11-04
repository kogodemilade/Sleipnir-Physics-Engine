cmake_minimum_required(VERSION 3.10)
project(CycloneSubset)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O2")

include_directories(include)
file(GLOB SOURCES "src/*.cpp")

add_executable(physics_test ${SOURCES})