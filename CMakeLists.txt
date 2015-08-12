cmake_minimum_required(VERSION 2.4.6)
cmake_policy(SET CMP0005 NEW)
cmake_policy(SET CMP0012 NEW)
cmake_policy(SET CMP0003 NEW)

project(ros4rsb)

find_package(catkin REQUIRED COMPONENTS message_generation 
                                        tf
                                        dynamic_reconfigure
                                        costmap_2d
                                        actionlib
                                        waving_detection
                                        std_msgs 
                                        geometry_msgs 
                                        sensor_msgs
                                        nav_msgs
                                        diagnostic_msgs
                                        actionlib_msgs
                                        roscpp 
                                        roslib)

catkin_package(CATKIN_DEPENDS message_runtime
                              tf
                              dynamic_reconfigure
                              costmap_2d
                              actionlib
                              waving_detection
                              std_msgs
                              geometry_msgs
                              sensor_msgs
                              nav_msgs
                              diagnostic_msgs
                              actionlib_msgs
                              roscpp
                              roslib
                              )

## Include catkin deps
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS system signals)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
MESSAGE(STATUS "Eigen: ${EIGEN3_INCLUDE_DIR}")

# Try to find RSB and its components
find_package(RSC QUIET REQUIRED)
find_package(RSB QUIET REQUIRED)
find_package(RST QUIET REQUIRED COMPONENTS sandbox)
find_package(rst-converters QUIET REQUIRED)
find_package(ProtocolBuffers QUIET REQUIRED)

message(STATUS "RSC version: ${RSC_VERSION}")
message(STATUS "RSB version: ${RSB_VERSION}")
message(STATUS "RST version: ${RST_VERSION}")
message(STATUS "RST_CONVERTERS version: ${RST_CONVERTERS_VERSION}")
# FIX FOR RST VERSION "1.0.0" aka "0.7.100"
if(${RST_VERSION} EQUAL "1.0.0" OR ${RST_VERSION} EQUAL "0.7.100") 
    set(CONFIG_DIR ${RST_CONFIG_DIR})
    find_package(RST QUIET REQUIRED COMPONENTS sandbox)
endif()

include_directories(SYSTEM ${RSC_INCLUDE_DIRS} ${RSB_INCLUDE_DIRS} ${RST_INCLUDE_DIRS} ${RST_CONVERTERS_INCLUDE_DIRS})
link_directories(${RSC_LIBRARY_DIRS} ${RSB_LIBRARY_DIRS} ${RST_CONVERTERS_LIBRARY_DIRS})
add_definitions(${RST_CFLAGS})

set(CMAKE_MODULE_PATH "${RSC_CMAKE_MODULE_PATH}" "${CMAKE_MODULE_PATH}")

IF(NOT PROTOBUF_LIBRARY)
    MESSAGE(FATAL_ERROR "protobuf library not found")
ENDIF()
IF(NOT PROTOBUF_PROTOC_EXECUTABLE)
    MESSAGE(FATAL_ERROR "protoc executable not found")
ENDIF()

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/src)

set(SOURCES src/ros4rsb.cpp
            src/publishers/LaserDataPublisher.cpp
            src/publishers/OdometryDataPublisher.cpp
            src/publishers/SpeedDataPublisher.cpp
            src/publishers/StallDataPublisher.cpp
            src/publishers/SlamMapPublisher.cpp
            src/publishers/SlamPosPublisher.cpp
            src/publishers/GlobalPlanPublisher.cpp
            src/publishers/WaveDetectionPublisher.cpp
            src/listeners/BoxListener.cpp
            #src/actuators/VelocityCommander.cpp
            #src/servers/NavigationServer.cpp         
            src/actuators/VelocityCommander.cpp
            src/servers/Costmap.cpp
            src/servers/NavigationServer.cpp
            )

add_executable(ros4rsb ${SOURCES})
target_link_libraries(ros4rsb ${RSC_LIBRARIES} ${RSB_LIBRARIES} ${RST_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
