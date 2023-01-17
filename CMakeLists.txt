# Structure for a ROS CMakeLists.txt file

# 1) Required CMake Version (cmake_minimum_required)
# 2) Package Name (project())
# 3) Find other CMake/Catkin packages needed for build (find_package())
# 4) Enable Python module support (catkin_python_setup())
# 5) Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
# 6) Invoke message/service/action generation (generate_messages())
# 7) Specify package build info export (catkin_package())
# 8) Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
# 9) Tests to build (catkin_add_gtest())
# 10) Install rules (install())

cmake_minimum_required(VERSION 3.16)
project(sgpr_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(PCL REQUIRED)
find_package(Armadillo REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(Matplot++ REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp message_generation std_msgs sensor_msgs
)

add_service_files(
  FILES
  Eigenvalues.srv
  PointClouds.srv
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy
)

catkin_install_python(PROGRAMS
  scripts/evaluation_service.py
  scripts/point_cloud_viz_service.py
  scripts/histogram_viz_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Main node files and executable
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS "src/*.cpp")

add_executable(sgpr_ros ${SRCS})

target_link_libraries(sgpr_ros PRIVATE
    nlohmann_json::nlohmann_json
    Matplot++::matplot
    glfw
    ${PCL_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_include_directories(sgpr_ros PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/GUI
    ${PCL_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
