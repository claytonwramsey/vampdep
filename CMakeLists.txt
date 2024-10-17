cmake_minimum_required(VERSION 3.16...3.22)

project(
    vampdep
    VERSION 0.1
    LANGUAGES CXX
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${VAMP_ARCH} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 ${VAMP_FAST_ARGS}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${VAMP_ARCH} -Wall -Wextra")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -g")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 ${VAMP_FAST_ARGS}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set up CPM
file(
  DOWNLOAD
  https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.1/CPM.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
  EXPECTED_HASH SHA256=117cbf2711572f113bab262933eb5187b08cfc06dce0714a1ee94f2183ddc3ec
)
set(CPM_USE_LOCAL_PACKAGES ON)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

CPMAddPackage("gh:KavrakiLab/vamp#bd5d218684c1608a61306537e4532932364e3411")
set(VAMP_INCLUDE_DIRS ${vamp_SOURCE_DIR}/src/impl)

CPMAddPackage("gh:kavrakilab/nigh#97130999440647c204e0265d05a997dbd8da4e70")
set(NIGH_INCLUDE_DIRS ${nigh_SOURCE_DIR}/src)

CPMAddPackage("gh:orlp/pdqsort#b1ef26a55cdb60d236a5cb199c4234c704f46726")
set(PDQSORT_INCLUDE_DIRS ${pdqsort_SOURCE_DIR})

list(APPEND VAMP_INCLUDE_DIRS ${NIGH_INCLUDE_DIRS} ${PDQSORT_INCLUDE_DIRS})

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  CPMAddPackage("gh:lemire/SIMDxorshift#857c1a01df53cf1ee1ae8db3238f0ef42ef8e490")
  list(APPEND VAMP_EXT_SOURCES
    ${SIMDxorshift_SOURCE_DIR}/src/simdxorshift128plus.c
    ${SIMDxorshift_SOURCE_DIR}/src/xorshift128plus.c)
  list(APPEND VAMP_INCLUDE_DIRS ${SIMDxorshift_SOURCE_DIR}/include)
endif()

include_directories(include)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(test src/test.cpp)
target_link_libraries(test PRIVATE Eigen3::Eigen)
target_include_directories(
    test SYSTEM PRIVATE
    ${VAMP_INCLUDE_DIRS}
)
