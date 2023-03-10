# project
cmake_minimum_required(VERSION 3.16)

project(driverstation VERSION 0.2)
set(CMAKE_EXPORT_COMPILE_COMMANDS On)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)

find_package(raylib REQUIRED)
find_library(NAMES ssh REQUIRED)

add_subdirectory("src")
add_subdirectory("include")

target_include_directories(${PROJECT_NAME} PUBLIC "src" "include" "/usr/local/include")
target_link_libraries(${PROJECT_NAME} PRIVATE raylib)
target_link_libraries(${PROJECT_NAME} PRIVATE ssh)
target_link_libraries(${PROJECT_NAME} PRIVATE glib-2.0)

set(ASSETS_SOURCE_DIR "${PROJECT_SOURCE_DIR}/assets")
set(ASSETS_DESTINATION_DIR "${CMAKE_CURRENT_BINARY_DIR}")
file(COPY "${ASSETS_SOURCE_DIR}" DESTINATION "${ASSETS_DESTINATION_DIR}")
