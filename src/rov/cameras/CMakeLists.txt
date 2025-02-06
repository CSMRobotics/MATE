cmake_minimum_required(VERSION 3.8)
project(cameras)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GST REQUIRED gstreamer-1.0>=1.4
    gstreamer-sdp-1.0>=1.4
    gstreamer-video-1.0>=1.4
    gstreamer-app-1.0>=1.4)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(cameras src/cameras.cpp)
target_include_directories(cameras PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cameras PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(cameras PRIVATE ${GST_INCLUDE_DIRS})
target_link_libraries(cameras ${GST_LIBRARIES})

install(TARGETS cameras
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
