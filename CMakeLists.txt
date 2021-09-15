cmake_minimum_required(VERSION 3.16.3)

set(SOURCE_DIR
    src
)

set(SOURCE
    ${SOURCE_DIR}/main.cpp
)

#set(LIBRARIES
#   A
#   B
#)

project(ROV)


add_executable(${PROJECT_NAME} ${SOURCE})