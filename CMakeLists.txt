cmake_minimum_required(VERSION  3.16.3)

project(ROV)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)

file(GLOB all_SRCS
    "${PROJECT_SOURCE_DIR}/include/*.hpp"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

# library directories
include_directories(${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/libs/CSMUtil ${PROJECT_SOURCE_DIR}/libs/JHPWMPCA9685)

add_executable(ROV ROV.cpp ${all_SRCS})

# link the libraries to project
target_link_libraries(ROV ${OpenCV_LIBS} CSMUtil JHPWMPCA9685)