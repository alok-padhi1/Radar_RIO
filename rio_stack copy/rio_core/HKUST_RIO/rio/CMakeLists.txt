cmake_minimum_required(VERSION 3.0.2)
project(rio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-std=c++17 -O3)

# Find Non-ROS Dependencies
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(gflags REQUIRED)

# Mock glog::glog target because libunwind is missing
add_library(glog::glog SHARED IMPORTED)
set_target_properties(glog::glog PROPERTIES IMPORTED_LOCATION "/usr/lib/x86_64-linux-gnu/libglog.so")

find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)

# Try to find Backward (optional, for stacktraces)
find_package(Backward QUIET)

###########
## Build ##
###########

FILE(GLOB_RECURSE RIO_SOURCE_FILES "src/*.cpp")
add_executable(rio_node
    node/rio.cpp
    node/zmqWarper.cpp
    ${RIO_SOURCE_FILES}
)

target_include_directories(rio_node
PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    node
    src/data_manager
    src/factor
    src/frame_type
    src/frontend
    src/residual
    src/state
    src/utility
    src/manifold
    /usr/include/jsoncpp
)

target_link_libraries(rio_node
PRIVATE
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    ${PCL_LIBRARIES}
    Threads::Threads
    zmq
    jsoncpp
)
