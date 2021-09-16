cmake_minimum_required(VERSION 3.16.3)

project(ROV)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("include/")

add_subdirectory(src)
add_subdirectory(apps)

target_link_libraries(ROV source ${OpenCV_LIBS})