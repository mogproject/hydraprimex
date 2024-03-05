cmake_minimum_required(VERSION 3.14)

# Avoid warning about DOWNLOAD_EXTRACT_TIMESTAMP in CMake 3.24:
if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  cmake_policy(SET CMP0135 NEW)
endif()

# options
option(BUILD_TESTS "Build test programs" OFF)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# optimize kissat for UNsat instances
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -DNDEBUG -DUNSAT -DNPROOFS -DQUIET")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fopenmp -lpthread")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fno-omit-frame-pointer -fsanitize=leak -O0 -gdwarf-4")
  # Note: `-gdwarf-4` is a hack for enabling pretty printing with LLDB on MacOS.
  #       ref: https://stackoverflow.com/questions/66225829/why-lldb-pretty-printers-doesnt-work-for-gnu-gcc-compiled-program-on-macos
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -funroll-loops -O3")
endif ()

message("Compiler: ${CMAKE_CXX_COMPILER}")
message("Options: ${CMAKE_CXX_FLAGS}")

# source files
file(GLOB HYDRA_SOLVER_SRC hydraprimex.cpp)
file(GLOB_RECURSE MAIN_SRC
  algorithm/*.[ch]pp
  app/*.[ch]pp
  ds/*.[ch]pp
  generator/*.[ch]pp
  readwrite/*.[ch]pp
  sat/*.[ch]pp
  util/*.[ch]pp
)

# external C sources
file(GLOB KISSAT_SRC ../../../external/kissat/*.c)
set_source_files_properties( ${KISSAT_SRC} PROPERTIES LANGUAGE C )

include_directories(
  .
  ../../../external
)

# project
project(hydrax)

# build options
if (NOT DEFINED PROFILE_ON)
  set(PROFILE_ON false)
endif ()

add_compile_definitions(PROFILE_ON=${PROFILE_ON})

# tests with GoogleTest
if (BUILD_TESTS)
  add_subdirectory(../../test/cpp ../test)
else ()
  add_executable(hydrax ${HYDRA_SOLVER_SRC} ${KISSAT_SRC} ${MAIN_SRC})
endif ()
