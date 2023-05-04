# project
cmake_minimum_required(VERSION 3.16)

project(driverstation_gui VERSION 0.2)
set(CMAKE_EXPORT_COMPILE_COMMANDS On)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS Off)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()

# Make compiler pedantic
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(raylib REQUIRED VERSION 4.2.0)
find_library(NAME ssh REQUIRED)
find_library(NAME raygui REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
)

target_link_libraries(${PROJECT_NAME} raylib)
target_link_libraries(${PROJECT_NAME} ssh)
target_link_libraries(${PROJECT_NAME} glib-2.0)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
 DESTINATION include/)

# TODO: THIS WILL CAUSE PATHS TO NOT WORK CORRECT UNLESS YOU
# APPEND get_package_share_directory('${PROJECT_NAME}')
# BEFORE THE DESIRED PATH
# found in #include <ament_index_cpp/get_package_share_directory.hpp>
install(DIRECTORY "${PROJECT_SOURCE_DIR}/assets"
  DESTINATION share/${PROJECT_NAME})

install(FILES "${PROJECT_SOURCE_DIR}/../launch/driverstation_gui_launch.py"
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
