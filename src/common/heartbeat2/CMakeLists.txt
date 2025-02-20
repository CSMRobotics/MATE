cmake_minimum_required(VERSION 3.8)
project(heartbeat2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(csm_common_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost 1.40 REQUIRED COMPONENTS container)

# add executables for ros to recognize
add_executable(heartbeat_client src/heartbeat_client.cpp)
target_include_directories(heartbeat_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  heartbeat_client
  rclcpp
  csm_common_interfaces
  std_srvs
)
target_include_directories(heartbeat_client PUBLIC ${Boost_INCLUDE_DIR})
target_link_libraries(heartbeat_client ${Boost_LIBRARIES})

add_executable(heartbeat_server src/heartbeat_server.cpp)
target_include_directories(heartbeat_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  heartbeat_server
  rclcpp
  csm_common_interfaces
  std_srvs
)

install(TARGETS heartbeat_client
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS heartbeat_server
  DESTINATION lib/${PROJECT_NAME})


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
