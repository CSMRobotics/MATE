add_library(jhpwmpca9685 ${CMAKE_CURRENT_SOURCE_DIR}/src/JHPWMPCA9685.cpp)
target_include_directories(jhpwmpca9685 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(JHPWMPCA9685_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)