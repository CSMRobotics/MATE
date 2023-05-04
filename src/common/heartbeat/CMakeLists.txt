cmake_minimum_required(VERSION 3.23.0)
project(heartbeat)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# if build type is debug or none, generate additional logger calls
if(CMAKE_BUILD_TYPE EQUAL "DEBUG" OR NOT CMAKE_BUILD_TYPE)
  message(WARNING "Build type is DEBUG or None, generating additional RCLCPP_Logger calls")
  add_definitions(-DDEBUG)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(csm_common_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_executable(listener src/listener.cpp)
target_include_directories(listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  listener
  rclcpp
  common_interfaces
  builtin_interfaces
)

add_executable(producer src/producer.cpp)
target_include_directories(producer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  producer
  rclcpp
  common_interfaces
  builtin_interfaces
)

install(TARGETS listener
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS producer
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include/)

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
