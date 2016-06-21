cmake_minimum_required(VERSION 2.8)
add_definitions(-std=c++11)

project( pioneerax )

find_package (Threads REQUIRED)
#ADDING ARIA
find_library(ARIA_LIBRARY NAMES Aria ArNetworking PATHS /usr/local/Aria/lib)
include_directories( /usr/local/Aria/include)
message(${ARIA_LIBRARY})
#ADDING CERES
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
#ADDING OPEN CV
find_package( OpenCV REQUIRED )
#INCLUDING FILES
include_directories(include)
include_directories(lib/odometry/include)
#ADDING SOURCE FILES
add_executable(pioneerax src/main.cpp
	src/robothandler
	src/circulartrajectory
	src/controller
	src/camhandler
	lib/odometry/src/epipolar
	lib/odometry/src/odometry)
#aria class handler
target_link_libraries(pioneerax ${ARIA_LIBRARY})
target_link_libraries(pioneerax ${CERES_LIBRARIES})
target_link_libraries(pioneerax dl)
target_link_libraries(pioneerax rt)
target_link_libraries(pioneerax ${OpenCV_LIBRARIES})

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O2")        ## Optimize
    set(CMAKE_EXE_LINKER_FLAGS "-s")  ## Strip binary
endif()
