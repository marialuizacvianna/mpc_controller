project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++14 -O3)

find_package(PythonLibs REQUIRED)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}" )

set(sources src/lib_nn.cpp src/lib_mpc.cpp src/main.cpp src/lib_io.cpp)

include_directories(/usr/local/include)
include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS glut GLU GL ${PYTHON_LIBRARIES})
