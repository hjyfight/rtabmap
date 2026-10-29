cmake_minimum_required(VERSION 3.5)
project(rtabmap_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288 (best-effort probes: not REQUIRED, a phantom miss under
  # emulated arm64 should not fail the build)
  find_library(
    builtin_interfaces__rosidl_generator_c_LIB NAMES builtin_interfaces__rosidl_generator_c
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
  )
  find_library(
    rcutils_LIB NAMES rcutils
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
  )
  find_library(
    crypto_LIB NAMES crypto
    PATHS "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(rtabmap_semantic_msgs REQUIRED)
find_package(rtabmap_sync REQUIRED)
find_package(tf2 REQUIRED)

find_package(RTABMap COMPONENTS gui REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# libraries
SET(Libraries
   cv_bridge::cv_bridge
   geometry_msgs::geometry_msgs
   rclcpp::rclcpp
   std_msgs::std_msgs
   std_srvs::std_srvs
   nav_msgs::nav_msgs
   rtabmap_msgs::rtabmap_msgs
   rtabmap_semantic_msgs::rtabmap_semantic_msgs
   rtabmap_sync::rtabmap_sync
   tf2::tf2
   rtabmap::gui
)
SET(AmentLibraries
   cv_bridge
   geometry_msgs
   rclcpp
   std_msgs
   std_srvs
   nav_msgs
   rtabmap_msgs
   rtabmap_semantic_msgs
   rtabmap_sync
   tf2
   RTABMap
)

###########
## Build ##
###########

add_executable(rtabmap_viz src/GuiNode.cpp src/GuiWrapper.cpp src/PreferencesDialogROS.cpp
  src/SemanticMainWindow.cpp include/${PROJECT_NAME}/SemanticMainWindow.h
  include/${PROJECT_NAME}/PreferencesDialogROS.h)
if("$ENV{ROS_DISTRO}" STRLESS "lyrical")
  ament_target_dependencies(rtabmap_viz ${AmentLibraries})
else()
  target_link_libraries(rtabmap_viz PRIVATE ${Libraries})
endif()
SET_TARGET_PROPERTIES(
  rtabmap_viz
   PROPERTIES
	   AUTOUIC ON
     AUTOMOC ON
     AUTORCC ON
)

add_executable(rgbd_image_viewer src/RGBDImageViewerNode.cpp src/rgbd_image_viewer.cpp include/${PROJECT_NAME}/rgbd_image_viewer.hpp)
if("$ENV{ROS_DISTRO}" STRLESS "lyrical")
  ament_target_dependencies(rgbd_image_viewer ${AmentLibraries})
else()
  target_link_libraries(rgbd_image_viewer PRIVATE ${Libraries})
endif()
SET_TARGET_PROPERTIES(
  rgbd_image_viewer
   PROPERTIES
	   AUTOUIC ON
     AUTOMOC ON
     AUTORCC ON
)

#############
## Install ##
#############

option(RTABMAP_SEMANTIC_GUI_CHECK "Build the native semantic rendering integration check" OFF)
if(RTABMAP_SEMANTIC_GUI_CHECK)
  add_executable(semantic_gui_check test/semantic_gui_check.cpp src/SemanticMainWindow.cpp
    include/${PROJECT_NAME}/SemanticMainWindow.h)
  ament_target_dependencies(semantic_gui_check ${AmentLibraries})
  set_target_properties(semantic_gui_check PROPERTIES AUTOMOC ON AUTOUIC ON AUTORCC ON)
  install(TARGETS semantic_gui_check DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS 
   rtabmap_viz
   rgbd_image_viewer
   DESTINATION lib/${PROJECT_NAME}
)

ament_package()
