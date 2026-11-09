cmake_minimum_required(VERSION 3.20)

project(PhysicsEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build raylib from the copy stored in this repository
add_subdirectory(external/raylib)

# Python 3.10 configuration for GPU Parallelism Bridge
set(PYTHON_ROOT "C:/Users/JUNAID/AppData/Local/Programs/Python/Python310")

# Build our application
add_executable(PhysicsEngine
    src/main.cpp
    src/physics/PhysicsWorld.cpp
    src/physics/GPUPhysicsWorld.cpp
    src/physics/RigidBody.cpp
    src/physics/Circle.cpp
    src/physics/Box.cpp
    src/Render/Renderer.cpp
)

target_include_directories(PhysicsEngine PRIVATE
    src
    "${PYTHON_ROOT}/include"
)

target_link_directories(PhysicsEngine PRIVATE
    "${PYTHON_ROOT}/libs"
)

# Link raylib and python310
target_link_libraries(PhysicsEngine PRIVATE 
    raylib
    python310
)
