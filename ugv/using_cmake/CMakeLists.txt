cmake_minimum_required(VERSION 2.8.12)

project(ugv)

find_package(DART 6.2.0 REQUIRED COMPONENTS collision-bullet CONFIG)

add_compile_options(-std=c++14)

include_directories(${DART_INCLUDE_DIRS})

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES})

# From this directory:
# $ mkdir build
# $ cd build
# $ set -x "DART_DIR" "/Users/ajc/.local/share/dart/cmake"; and cmake -DCMAKE_RULE_MESSAGES:BOOL=OFF -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON ..
# $ make --no-print-directory
# /Users/ajc/.local/share/dart/cmake/DARTConfig.cmake
