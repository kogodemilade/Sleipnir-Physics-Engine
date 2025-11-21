cmake_minimum_required(VERSION 3.10)
project(CycloneSubset)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O2")

include_directories(include)
file(GLOB SOURCES "src/*.cpp")

find_package(glfw3 REQUIRED)
add_executable(Sleipnir ${SOURCES})

# add_subdirectory(lib)
target_link_libraries(Sleipnir PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/libYgg.a
    glfw3
    GL
)
