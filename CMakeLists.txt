cmake_minimum_required(VERSION  3.16.3)

project(ROV)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)

file(GLOB all_SRCS
    "${PROJECT_SOURCE_DIR}/include/*.hpp"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

# include OpenCV
find_package(OpenCV REQUIRED)

# library directories
add_subdirectory("${PROJECT_SOURCE_DIR}/libs/CSMUtil")
add_subdirectory("${PROJECT_SOURCE_DIR}/libs/JHPWMPCA9685")
include_directories(${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/libs/CSMUtil ${PROJECT_SOURCE_DIR}/libs/JHPWMPCA9685)
link_directories(${PROJECT_SOURCE_DIR}/libs/CSMUtil ${PROJECT_SOURCE_DIR}/libs/JHPWMPCA9685)

# create executable
add_executable(ROV ROV.cpp ${all_SRCS})

# link the libraries to project
target_link_libraries(ROV ${OpenCV_LIBS} JHPWMPCA9685 CSMUtil pthread)
