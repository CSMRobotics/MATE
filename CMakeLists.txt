cmake_minimum_required(VERSION  3.16.3)

project(ROV)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    # uncomment to include tests
    ${PROJECT_SOURCE_DIR}/tests
)

file(GLOB all_SRCS
    "${PROJECT_SOURCE_DIR}/include/*.hpp"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
    # uncomment to include tests
    "${PROJECT_SOURCE_DIR}/tests/*.cpp"
)


# library directories
add_subdirectory("${PROJECT_SOURCE_DIR}/libs/CSMUtil")
add_subdirectory("${PROJECT_SOURCE_DIR}/libs/JHPWMPCA9685")
include_directories(${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/libs/CSMUtil ${PROJECT_SOURCE_DIR}/libs/JHPWMPCA9685)
link_directories(${PROJECT_SOURCE_DIR}/libs/CSMUtil ${PROJECT_SOURCE_DIR}/libs/JHPWMPCA9685)

# create executable
add_executable(ROV ROV.cpp ${all_SRCS})

# link the libraries to project
target_link_libraries(ROV ${OpenCV_LIBS} JHPWMPCA9685 CSMUtil i2c)
