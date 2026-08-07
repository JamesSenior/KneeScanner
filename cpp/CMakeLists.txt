cmake_minimum_required(VERSION 3.15)
project(cpp_knee_alignment)

include(FetchContent)

FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)

FetchContent_MakeAvailable(json)


#FetchContent_Declare(
#    cpd
#    GIT_REPOSITORY https://github.com/gadomski/cpd.git
#    GIT_TAG main
#)

find_package(Eigen3 REQUIRED)

#FetchContent_MakeAvailable(cpd)

set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE SOURCES src/*.cpp)

#file(GLOB_RECURSE CPD_SOURCES
#    ${cpd_SOURCE_DIR}/src/*.cpp
#)
#list(FILTER CPD_SOURCES EXCLUDE REGEX ".*fgt.*")

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})

add_executable(cpp_knee_alignment 
    ${SOURCES}
    #${CPD_SOURCES}
)

target_include_directories(cpp_knee_alignment PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include
    #${cpd_SOURCE_DIR}/include
)

target_link_libraries(cpp_knee_alignment PRIVATE Eigen3::Eigen PRIVATE nlohmann_json::nlohmann_json)