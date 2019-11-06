project(staticpendulum)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(SOURCE
pendulummap.cpp
pendulumsystem.cpp
main.cpp
)
add_subdirectory(integrators)
add_subdirectory(lodepng)
add_subdirectory(jsoncpp)
add_executable(${PROJECT_NAME} ${SOURCE})
