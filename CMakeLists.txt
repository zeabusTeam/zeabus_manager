cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_manager )

find_package( catkin REQUIRED COMPONENTS 
    roscpp
    zeabus_utility
)

find_package( zeabus_library REQUIRED COMPONENTS )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS roscpp
)

add_executable( switcher
    src/manager_switch.cpp
)
target_link_libraries( switcher
    ${zeabus_library_LIBRARIES}
)
