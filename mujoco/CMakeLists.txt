cmake_minimum_required(VERSION 3.10)

# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

# This line has to appear before 'PROJECT' in order to be able to disable incremental linking
set(MSVC_INCREMENTAL_DEFAULT ON)

set(MUJOCO_DEP_VERSION_lodepng
    b4ed2cd7ecf61d29076169b49199371456d4f90b
    CACHE STRING "Version of `lodepng` to be fetched."
)

project(
  mujoco_ros2
  VERSION 1.0.0
  )

enable_language(C)
enable_language(CXX)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(SimulateOptions)
include(SimulateDependencies)

include(CheckIPOSupported)
check_ipo_supported(RESULT USE_IPO) 

set(MUJOCO_SIMULATE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")
set(MUJOCO_SIMULATE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

if(MUJOCO_HARDEN)
    set(MUJOCO_SIMULATE_COMPILE_OPTIONS "${MUJOCO_SIMULATE_COMPILE_OPTIONS}" -fPIE -fPIC)
    set(MUJOCO_SIMULATE_LINK_OPTIONS "${MUJOCO_SIMULATE_LINK_OPTIONS}" -pie)
endif()

# Simulate library
add_library(platform_ui_adapter_ros2 OBJECT)
target_sources(
  platform_ui_adapter_ros2
  PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/glfw_adapter.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/glfw_dispatch.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/platform_ui_adapter.h
  PRIVATE 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/glfw_adapter.cc 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/glfw_dispatch.cc 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/platform_ui_adapter.cc
)
target_compile_options(platform_ui_adapter_ros2 PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_include_directories(
  platform_ui_adapter_ros2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                             $<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(platform_ui_adapter_ros2 PUBLIC mujoco::mujoco)
add_library(mujoco::platform_ui_adapter_ros2 ALIAS platform_ui_adapter_ros2)

add_library(libsimulate_ros2 STATIC $<TARGET_OBJECTS:platform_ui_adapter_ros2>)
set_target_properties(libsimulate_ros2 PROPERTIES OUTPUT_NAME mujoco_node)
add_library(mujoco::libsimulate_ros2 ALIAS libsimulate_ros2)
target_sources(
  libsimulate_ros2
  PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/simulate.h
  PRIVATE 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/simulate.cc 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/array_safety.h
)
target_include_directories(libsimulate_ros2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(libsimulate_ros2 PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_link_libraries(libsimulate_ros2 PUBLIC lodepng mujoco::platform_ui_adapter_ros2 mujoco::mujoco)
target_link_options(libsimulate_ros2 PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(communication REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(mujoco_node 
src/mujoco_node.cpp 
src/MuJoCoMessageHandler.cpp 
include/array_safety.h 
include/MuJoCoMessageHandler.h)

target_compile_features(mujoco_node PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})  # Require C99 and C++17
target_link_libraries(
  mujoco_node
  libsimulate_ros2
  mujoco::mujoco
  glfw
  Threads::Threads
  lodepng
  ${CMAKE_DL_LIBS}
)
ament_target_dependencies(mujoco_node sensor_msgs rclcpp communication geometry_msgs tf2 tf2_ros nav_msgs )
target_include_directories(mujoco_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_options(mujoco_node PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})

install(TARGETS mujoco_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
