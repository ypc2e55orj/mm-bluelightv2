cmake_minimum_required(VERSION 3.16)

project(test-maze)

file(
        GLOB SOURCES
        "main.cc"
        "data/*.inc"
        "../../parameters.h"
        "../../map.h"
        "../../search.h")

message("### maze-test ##")
foreach (SOURCE IN LISTS SOURCES)
    message("Add: ${SOURCE}")
endforeach ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++23 -Wall -Wextra -Wdouble-promotion -Wfloat-equal")

add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})