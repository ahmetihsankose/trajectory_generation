cmake_minimum_required(VERSION 3.16)

project(trajectory_generator
    DESCRIPTION "axis trajectory generator using with fir filter"
    LANGUAGES C CXX)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB HEADER_FILES *.h)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})



