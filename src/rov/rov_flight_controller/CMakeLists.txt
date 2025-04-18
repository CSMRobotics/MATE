cmake_minimum_required(VERSION 3.5)
project(rov_flight_controller)

add_compile_options(-g)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()

if(DEBUG_OUTPUT)
  message(INFO " Compiling with Debug Output")
  add_compile_definitions(DEBUG_OUTPUT=true)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rov_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_DISPLAY)
  find_package(SFML 2.5 REQUIRED COMPONENTS system window graphics)
  find_package(Boost 1.71.0 REQUIRED)
endif()

add_library(flight_controller_lib STATIC src/flight_controller.cpp)
add_executable(flight_controller src/main.cpp)

if(BUILD_DISPLAY)
  add_executable(flight_display src/flight_display.cpp)
endif()

target_include_directories(flight_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  flight_controller_lib
  "rov_interfaces"
  "rclcpp"
  "message_filters"
  "std_srvs"
  "sensor_msgs"
)
if(BUILD_DISPLAY)
  target_include_directories(flight_display PUBLIC
    ${Boost_INCLUDE_DIRS})
endif()

target_link_libraries(flight_controller_lib Eigen3::Eigen)
target_link_libraries(flight_controller flight_controller_lib)
if(BUILD_DISPLAY)
  target_link_libraries(flight_display flight_controller_lib sfml-system sfml-window sfml-graphics yaml-cpp ${Boost_LIBRARIES})
endif()

install(TARGETS flight_controller flight_controller_lib
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_DISPLAY)
  install(TARGETS flight_display
  DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_DISPLAY)
  install(TARGETS flight_display
    DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  
  find_package(ament_cmake_gtest REQUIRED)
  set(TESTFILES
   test/main.cpp
   test/test_simple.cpp
  )
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test flight_controller_lib) #TODO: fix
  install(TARGETS
   ${PROJECT_NAME}_test
   DESTINATION
   lib/${PROJECT_NAME}
  )


  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
