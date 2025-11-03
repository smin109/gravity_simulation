cmake_minimum_required(VERSION 3.16)
project(gravity_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(nbody_core
    src/physics.cpp
    src/csv_writer.cpp
)

target_include_directories(nbody_core PUBLIC include)

add_executable(nbody_cli app/nbody_cli.cpp)
target_link_libraries(nbody_cli PRIVATE nbody_core)