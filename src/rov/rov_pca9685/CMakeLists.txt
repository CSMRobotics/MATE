cmake_minimum_required(VERSION 3.5)
project(rov_pca9685)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rov_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_library(pca9685 src/PCA9685_Node.cpp src/ServoDriver.cpp src/PCA9685.cpp)
add_executable(pca9685_node src/main.cpp)
add_executable(pca9685_interactive_test src/main_interactive.cpp)
target_include_directories(pca9685 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(pca9685_node pca9685)
target_link_libraries(pca9685_interactive_test pca9685)

ament_target_dependencies(pca9685
  rclcpp
  rov_interfaces
)

install(TARGETS pca9685_node pca9685 pca9685_interactive_test
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)
install(DIRECTORY external/JHPWMPCA9685/include/
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
