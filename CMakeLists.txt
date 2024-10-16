cmake_minimum_required(VERSION 3.16...3.22)

project(
    vampdep
    VERSION 0.1
    LANGUAGES CXX
)

# set up CPM
file(
  DOWNLOAD
  https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.1/CPM.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
  EXPECTED_HASH SHA256=117cbf2711572f113bab262933eb5187b08cfc06dce0714a1ee94f2183ddc3ec
)
set(CPM_USE_LOCAL_PACKAGES ON)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

CPMAddPackage("gh:KavrakiLab/vamp#b56f0a93d86fd6874bf722c3378cd9b9946a3276")
set(VAMP_INCLUDE_DIRS ${vamp_SOURCE_DIR}/src/impl)

include_directories(include)


add_executable(test src/test.cpp)
target_include_directories(test SYSTEM PRIVATE ${VAMP_INCLUDE_DIRS})
