cmake_minimum_required(VERSION 3.16)

project(trajectory_generator
    DESCRIPTION "axis trajectory generator using with fir filter"
    LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB HEADER_FILES *.h)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})




