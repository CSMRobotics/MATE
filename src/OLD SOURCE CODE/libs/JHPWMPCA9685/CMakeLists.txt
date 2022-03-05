add_library( JHPWMPCA9685
    JHPWMPCA9685.h
    JHPWMPCA9685.cpp
)

target_include_directories (JHPWMPCA9685 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(JHPWMPCA9685 i2c)