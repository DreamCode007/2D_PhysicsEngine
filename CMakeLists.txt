cmake_minimum_required(VERSION 3.20)

project(PhysicsEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build raylib from the copy stored in this repository
add_subdirectory(external/raylib)

# Build our application
add_executable(PhysicsEngine
    src/main.cpp
    src/physics/PhysicsWorld.cpp
    src/physics/RigidBody.cpp
    src/physics/Circle.cpp
    src/physics/Box.cpp
    src/Render/Renderer.cpp
)

# Link raylib
target_link_libraries(PhysicsEngine PRIVATE raylib)