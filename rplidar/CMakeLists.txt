cmake_minimum_required(VERSION 2.8.3)
project(rplidar)

find_package(catkin REQUIRED COMPONENTS sensor_msgs roscpp)

catkin_package()

# If your rplidar sdk path has been changed, please modify the following path.
set(RPLIDAR_DIR /home/exbot/rplidar_sdk_v1.4.2/sdk)
set(RPLIDAR_LIB ${RPLIDAR_DIR}/output/Linux/Release/librplidar_sdk.a)
set(RPLIDAR_INCLUDE ${RPLIDAR_DIR}/sdk/include)

include_directories(${RPLIDAR_INCLUDE} ${catkin_INCLUDE_DIRS})

# Build nodes
add_executable(rplidar nodes/rplidar.cpp)
target_link_libraries(rplidar ${RPLIDAR_LIB} ${catkin_LIBRARIES})