add_library( BNO055 STATIC
    bno055.h
    bno055.cpp
)

target_include_directories(BNO055 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})