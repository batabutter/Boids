cmake_minimum_required(VERSION 3.23)

project(Boids)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
  raylib
  GIT_REPOSITORY  https://github.com/raysan5/raylib.git
  GIT_TAG         6.0
)
FetchContent_MakeAvailable(raylib)

add_executable(main)
target_sources(main
  PRIVATE
    src/main.cpp
)

add_compile_options(-Wall -Wpedantic)

target_link_libraries(main PRIVATE raylib)
