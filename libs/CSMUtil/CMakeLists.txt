add_library( CSMUtil STATIC
    CSMUtil.hpp
    CSMUtil.cpp
    CSMBNO055.hpp
    CSMBNO055.cpp
    BAR02.hpp
    BAR02.cpp
)

target_include_directories (CSMUtil PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})