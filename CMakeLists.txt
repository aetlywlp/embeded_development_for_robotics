

cmake_minimum_required(VERSION 3.8)

include(cmake/arm_toolchain.cmake)
include(cmake/build_helper.cmake)
include(cmake/clang_format.cmake)
include(cmake/doxygen.cmake)

project(FOCUS_Embedded_2025)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

add_subdirectory(boards)
add_subdirectory(shared)
add_subdirectory(shared_DMMC02)
add_subdirectory(shared_f103c8)
add_subdirectory(vehicles)
add_subdirectory(examples)
