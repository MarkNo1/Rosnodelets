cmake_minimum_required(VERSION 3.0)
project(rosnodelets)

# Include ros compliancy
include(ros_compliancy.cmake)

# Find needed packages with catkin
find_package(catkin REQUIRED COMPONENTS
        nodelet)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#        DEPENDS
        CATKIN_DEPENDS   nodelet
        INCLUDE_DIRS include
#        LIBRARIES
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


##############
## Manager  ##
##############
add_executable( nodelet_manager app/nodelet_manager.cc src/manager/NodeletManager.cc)
add_dependencies(nodelet_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nodelet_manager ${catkin_LIBRARIES})


##############
## Nodelets ##
##############
############################  Test  ############################
#|End Nodelets|


# Python scripts
install(PROGRAMS scripts/add_nodelet
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
