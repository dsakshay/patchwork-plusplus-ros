cmake_minimum_required(VERSION 3.5)
project(patchworkpp)

add_compile_options(-std=c++17)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


if( CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
  message(STATUS "------------------------------------------")
  message(STATUS "patchworkpp is being built using CATKIN.")
  message(STATUS "------------------------------------------")


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        roslaunch
        cv_bridge
        pcl_conversions
        pcl_ros
        geometry_msgs
        laser_geometry
        sensor_msgs
        message_generation
        jsk_recognition_msgs
)

find_package(OpenCV REQUIRED)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
)

add_executable(offline_kitti src/offline_kitti.cpp)
target_link_libraries(offline_kitti ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(offline_kitti patchworkpp_generate_messages_cpp)

add_executable(demo src/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(demo patchworkpp_generate_messages_cpp)

add_executable(video src/video.cpp)
target_link_libraries(video ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(video patchworkpp_generate_messages_cpp)


# ==== Install ====
install(TARGETS demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

elseif( DEFINED ENV{AMENT_PREFIX_PATH})
message(STATUS "------------------------------------------")
message(STATUS "patchworkpp is being built using AMENT.")
message(STATUS "------------------------------------------")

add_definitions(-DCOMPILED_WITH_AMENT)

if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fpn_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL 1.7 REQUIRED QUIET COMPONENTS common features filters io segmentation surface)
find_package(pcl_msgs REQUIRED)
find_package(pcl_ros REQUIRED)


set(ament_dependencies
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  std_msgs
  fpn_msgs
  tf2
  sensor_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
)


install(
  DIRECTORY include/
  DESTINATION include
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_library(patchworkpp SHARED src/patchworkpp_2.cpp)
ament_target_dependencies(patchworkpp ${ament_dependencies})
target_link_libraries(patchworkpp ${PCL_LIBRARIES})
ament_target_dependencies(patchworkpp pcl_conversions)


install(
  TARGETS patchworkpp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


add_executable(patchworkpp_2
  src/patchworkpp_2_node.cpp
)

ament_target_dependencies(patchworkpp_2 ${ament_dependencies})
target_link_libraries(patchworkpp_2 patchworkpp ${ament_INCLUDE_DIRS} ${PCL_LIBRARIES})

install(TARGETS patchworkpp_2
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
INCLUDES DESTINATION include
)

install(
   DIRECTORY launch rviz config
   DESTINATION share/${PROJECT_NAME}
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime rclcpp)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(patchworkpp)


ament_package()

endif()