# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8.3)

# Declare the name of the CMake Project
project(pcan_topics)

# Find and get all the information about the roscpp package
find_package( catkin REQUIRED COMPONENTS roscpp  libpcan)

#This function must be called before declaring any targets with add_library() or add_executable(). The function has 5 optional arguments: 
catkin_package()

# Define an execuable target called pcan_receive 
add_executable(pcan_receive src/pcan_receive_node.cpp src/pcan_receive.cpp src/common.cpp)

# Link the pcan_receive target against the libraries used by roscpp
target_link_libraries(pcan_receive   ${catkin_LIBRARIES})
target_link_libraries(pcan_receive   ${roscpp_LIBRARIES})

# Define an execuable target called pcan_transmit
add_executable(pcan_transmit src/pcan_transmit_node.cpp src/pcan_transmit.cpp src/common.cpp)

# Link the pcan_transmit target against the libraries used by roscpp
target_link_libraries(pcan_transmit   ${catkin_LIBRARIES})
target_link_libraries(pcan_receive   ${roscpp_LIBRARIES})


