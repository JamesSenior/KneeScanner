cmake_minimum_required(VERSION 3.15)
project(cpp_knee_alignment)

include(FetchContent)

FetchContent_Declare(
    cpd
    GIT_REPOSITORY https://github.com/gadomski/cpd.git
    GIT_TAG main
)

list(APPEND CMAKE_PREFIX_PATH "C:/Libraries/eigen-install")

FetchContent_MakeAvailable(cpd)

set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE SOURCES src/*.cpp)

file(GLOB_RECURSE CPD_SOURCES
    ${cpd_SOURCE_DIR}/src/*.cpp
)
list(FILTER CPD_SOURCES EXCLUDE REGEX ".*fgt.*")

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})

add_executable(cpp_knee_alignment 
    ${SOURCES}
    ${CPD_SOURCES}
)

target_include_directories(cpp_knee_alignment PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include
    C:/Libraries/eigen-5.0.0
    ${cpd_SOURCE_DIR}/include
)
