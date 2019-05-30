
if("$ENV{ROS_DISTRO}" STREQUAL "indigo") 
    cmake_minimum_required(VERSION 2.8.12)

    add_compile_options(-std=c++11)
    set(CMAKE_CXX_STANDARD 11)

#     find_package(Qt4 REQUIRED QtCore QtNetwork)
#     include(${QT_USE_FILE})
#     add_definitions( ${QT_DEFINITIONS} )
#     
#     macro(CUSTOM_QT_WRAP_CPP )
#         QT4_WRAP_CPP(${ARGN})
#     endmacro()
#     
#     set(CUSTOM_QT_LIBRARIES ${QT_LIBRARIES})
    
elseif("$ENV{ROS_DISTRO}" STREQUAL "kinetic") 
    cmake_minimum_required(VERSION 3.0)

    set(CMAKE_CXX_STANDARD 14)

#     find_package(Qt5 REQUIRED Core Network)
#     include_directories(${Qt5Core_INCLUDE_DIRS})
#     add_definitions(${Qt5Core_DEFINITIONS})
#     
#     macro(CUSTOM_QT_WRAP_CPP )
#         QT5_WRAP_CPP(${ARGN})
#     endmacro()
#     
#     set(CUSTOM_QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Network_LIBRARIES})

    
elseif("$ENV{ROS_DISTRO}" STREQUAL "melodic") 
    cmake_minimum_required(VERSION 3.10)

    set(CMAKE_CXX_STANDARD 17)

#     find_package(Qt5 REQUIRED Core Network)
#     include_directories(${Qt5Core_INCLUDE_DIRS})
#     add_definitions(${Qt5Core_DEFINITIONS})
#     
#     macro(CUSTOM_QT_WRAP_CPP )
#         QT5_WRAP_CPP(${ARGN})
#     endmacro()
#     
#     set(CUSTOM_QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Network_LIBRARIES})

else()
    message(FATAL_ERROR "ROS version not set!!")
endif()

