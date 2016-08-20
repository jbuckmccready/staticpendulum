project(staticpendulum)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -target x86_64-w64-mingw32 -Wall -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffast-math -march=native")
set(SOURCE
pendulummap.cpp
pendulumsystem.cpp
main.cpp
)
add_subdirectory(integrators)
add_subdirectory(lodepng)
add_subdirectory(jsoncpp)
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")
add_executable(${PROJECT_NAME} ${SOURCE})
