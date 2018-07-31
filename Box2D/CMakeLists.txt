# Specify the minimum version for CMake

cmake_minimum_required(VERSION 2.8)

# Project's name

project(Box2D)
# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/dist/lib)

# The following folder will be included
include_directories("${PROJECT_SOURCE_DIR}")

file(GLOB_RECURSE headers	Box2D/*.h)
file(GLOB_RECURSE sources 	Box2D/*.cpp)
list(APPEND sources ${headers})

add_library(Box2D ${sources})

set_property(TARGET Box2D PROPERTY CXX_STANDARD 11)
target_compile_options(Box2D PUBLIC -Wall -std=c++11)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

ADD_CUSTOM_COMMAND(
    TARGET Box2D
    POST_BUILD
    COMMAND rsync -qalW --del --include '*.h' --exclude '*.a' --exclude '*.cpp' --exclude '*.c' ${CMAKE_SOURCE_DIR}/Box2D/ ${CMAKE_BINARY_DIR}/dist/include/
)

install ( TARGETS Box2D DESTINATION lib )
install ( DIRECTORY ${CMAKE_BINARY_DIR}/dist/include/
	  DESTINATION include/Box2D
	  FILES_MATCHING PATTERN "*.h*" )

