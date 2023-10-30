cmake_minimum_required(VERSION 3.5)
project(rov_cameras)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(DEBUG_OUTPUT)
  add_compile_definitions(DEBUG_OUTPUT=true)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(rov_cameras src/main.cpp src/mjpeg_camera.cpp src/h264_camera.cpp src/camera_to_file.cpp)
target_include_directories(rov_cameras PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(
  rov_cameras
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "cv_bridge"
  "compressed_image_transport"
)

target_link_libraries(rov_cameras ${OpenCV_LIBS} udev pthread)

install(TARGETS rov_cameras
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME})

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
