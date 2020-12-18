project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++14 -O3)

find_package(PythonLibs REQUIRED)
find_package(OpenCV REQUIRED)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}" )

set(sources src/lib_nn.cpp src/lib_mpc.cpp  src/lib_io.cpp src/lib_trajectory.cpp src/main.cpp)
#src/scan_simulator_2d.cpp src/distance_transform.cpp src/lib_nn.cpp src/lib_mpc.cpp src/lib_io.cpp
include_directories(/usr/local/include)
include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(mpc ${sources})

target_link_libraries(mpc ipopt glut GLU GL ${PYTHON_LIBRARIES} ${OpenCV_LIBS})
