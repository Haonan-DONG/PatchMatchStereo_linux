cmake_minimum_required(VERSION 3.4)
project(patchmatchstereo)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

file(GLOB SRC_CODE /PatchMatchStereo/**)

add_library(pms ${PROJECT_SOURCE_DIR}/PatchMatchStereo/PatchMatchStereo.cpp)
add_library(pmsp ${PROJECT_SOURCE_DIR}/PatchMatchStereo/pms_propagation.cpp)
add_library(pmsu ${PROJECT_SOURCE_DIR}/PatchMatchStereo/pms_util.cpp)

add_executable(main ${PROJECT_SOURCE_DIR}/PatchMatchStereo/main.cpp)

target_link_libraries(main ${OpenCV_LIBS} pms pmsp pmsu)