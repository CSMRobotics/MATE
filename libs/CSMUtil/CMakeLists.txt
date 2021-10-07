add_library( CSMUtil STATIC
    CSMUtil.hpp
    CSMUtil.cpp
)

target_include_directories (CSMUtil PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})