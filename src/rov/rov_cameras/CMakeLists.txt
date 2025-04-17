cmake_minimum_required(VERSION 3.8)
project(rov_cameras)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rov_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GST REQUIRED gstreamer-1.0>=1.4
    gstreamer-sdp-1.0>=1.4
    gstreamer-video-1.0>=1.4
    gstreamer-app-1.0>=1.4)
pkg_search_module(GST_RTSP REQUIRED gstreamer-rtsp-server-1.0)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(rov_cameras src/cameras.cpp src/main.cpp)
target_include_directories(rov_cameras PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rov_cameras PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rov_cameras PRIVATE ${GST_INCLUDE_DIRS} ${GST_RTSP_INCLUDE_DIRS})
target_link_libraries(rov_cameras ${GST_LIBRARIES} ${GST_RTSP_LIBRARIES} gstrtsp-1.0 gstsdp-1.0 gstreamer-1.0 gio-2.0 gobject-2.0 glib-2.0)

ament_target_dependencies(rov_cameras
  rclcpp
  rov_interfaces
)

install(TARGETS rov_cameras
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

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
