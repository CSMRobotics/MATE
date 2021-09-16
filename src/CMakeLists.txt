
add_library(source 
components/Drive.cpp
components/Manipulator.cpp

${HEADER_LIST}
)

target_include_directories(source PUBLIC ../include)

target_compile_features(source PUBLIC cxx_std_17)

source_group(
  TREE "${PROJECT_SOURCE_DIR}/include"
  PREFIX "Header Files"
  FILES ${HEADER_LIST})