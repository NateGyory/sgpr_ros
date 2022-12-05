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
find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy
)

file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS "src/*.cpp")

add_executable(sgpr_ros ${SRCS})
target_link_libraries(sgpr_ros PRIVATE ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES} nlohmann_json::nlohmann_json Matplot++::matplot ${catkin_LIBRARIES})
target_include_directories(sgpr_ros PRIVATE ${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include ${ARMADILLO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
