add_library( CSMUtil STATIC
    CSMUtil.hpp
    CSMUtil.cpp
    CSMBNO055.hpp
    CSMBNO055.cpp
    Quaternion.hpp
    Vector3.hpp
    BMR4800116_005.hpp
    BMR4800116_005.cpp
    Matrix.hpp
)

target_include_directories (CSMUtil PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})