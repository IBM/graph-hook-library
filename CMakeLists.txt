# Copyright (c) 2025 IBM
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.16.0)
project(ghl C CXX)

# Add cmake directory to search for custom cmake functions
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# CXX options
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb -O0")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -flto -funroll-loops -DNDEBUG")
endif()
option(DO_DEBUG "Enabling debug mode" OFF)
option(GHL_SERIALIZATION "Build serialization code" OFF)
option(GHL_BINDINGS "Build python bindings" ON)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(GHL_MASTER_PROJECT ON)
else()
  set(GHL_MASTER_PROJECT OFF)
endif()

if(DO_DEBUG)
  add_definitions(-DDO_DEBUG)
  set(CMAKE_BUILD_TYPE Debug)
else()
  set(CMAKE_BUILD_TYPE Release)
endif()

if(WIN32)
  if(DO_DEBUG)
    set(CMAKE_CXX_FLAGS_DEBUG "/DDEBUG:FULL /Zi /EHsc /std:c++17 /bigobj /O0 /Oy- /D_CRT_SECURE_NO_WARNINGS")
  endif()
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(cmake/headers.cmake)
include(cmake/CPM.cmake)
include(cmake/GitUtils.cmake)

# boost
set(BOOST_INCLUDE_LIBRARIES "graph")

if(NOT TARGET boost_graph)
  CPMAddPackage(
    NAME Boost
    VERSION 1.86.0
    URL https://github.com/boostorg/boost/releases/download/boost-1.86.0/boost-1.86.0-cmake.tar.xz
    OPTIONS "BOOST_ENABLE_CMAKE ON"
  )
else()
  message("Found previous target boost_graph. Attempting to use to compile GHL.")
endif()

# Apply VF3 Patch
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vf3_patch.patch")
  message(STATUS "Applying custom VF3 patch...")

  # Check if patch has already been applied by looking for our new file
  set(NEW_BOOST_FILE "${Boost_SOURCE_DIR}/libs/graph/include/boost/graph/vf3_sub_graph_iso.hpp")

  if(NOT EXISTS "${NEW_BOOST_FILE}")
    # Find patch executable
    find_program(PATCH_EXECUTABLE patch)

    if(PATCH_EXECUTABLE)
      # Create the target directory first
      file(MAKE_DIRECTORY "${Boost_SOURCE_DIR}/libs/graph/include/boost/graph")

      # Test if patch can be applied (dry run)
      execute_process(
        COMMAND ${PATCH_EXECUTABLE} -p1 --dry-run -i "${CMAKE_CURRENT_SOURCE_DIR}/vf3_patch.patch"
        WORKING_DIRECTORY "${Boost_SOURCE_DIR}"
        RESULT_VARIABLE PATCH_DRY_RESULT
        OUTPUT_VARIABLE PATCH_DRY_OUTPUT
        ERROR_VARIABLE PATCH_DRY_ERROR
      )

      message(STATUS "Patch dry run result: ${PATCH_DRY_RESULT}")
      message(STATUS "Patch dry run output: ${PATCH_DRY_OUTPUT}")
      message(STATUS "Patch dry run error: ${PATCH_DRY_ERROR}")

      if(PATCH_DRY_RESULT EQUAL 0)
        # Actually apply the patch
        execute_process(
          COMMAND ${PATCH_EXECUTABLE} -p1 -i "${CMAKE_CURRENT_SOURCE_DIR}/vf3_patch.patch"
          WORKING_DIRECTORY "${Boost_SOURCE_DIR}"
          RESULT_VARIABLE PATCH_RESULT
          OUTPUT_VARIABLE PATCH_OUTPUT
          ERROR_VARIABLE PATCH_ERROR
        )

        if(PATCH_RESULT EQUAL 0)
          message(STATUS "Successfully applied VF3 patch")
          file(TOUCH "${CMAKE_CURRENT_BINARY_DIR}/vf3_patch_applied.marker")
        else()
          message(WARNING "Failed to apply patch: ${PATCH_ERROR}")
          # Fallback: copy file directly
          if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vf3_sub_graph_iso.hpp")
            configure_file(
              "${CMAKE_CURRENT_SOURCE_DIR}/vf3_sub_graph_iso.hpp"
              "${NEW_BOOST_FILE}"
              COPYONLY
            )
            message(STATUS "Copied file directly as fallback")
          endif()
        endif()
      else()
        message(WARNING "Patch dry run failed: ${PATCH_DRY_ERROR}")
        # Fallback: copy file directly
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vf3_sub_graph_iso.hpp")
          configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/vf3_sub_graph_iso.hpp"
            "${NEW_BOOST_FILE}"
            COPYONLY
          )
          message(STATUS "Copied file directly (patch failed)")
        endif()
      endif()
    else()
      message(WARNING "Patch command not found, cannot apply patch")
      # Fallback: copy file directly if available
      if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vf3_sub_graph_iso.hpp")
        file(MAKE_DIRECTORY "${Boost_SOURCE_DIR}/libs/graph/include/boost/graph")
        configure_file(
          "${CMAKE_CURRENT_SOURCE_DIR}/vf3_sub_graph_iso.hpp"
          "${NEW_BOOST_FILE}"
          COPYONLY
        )
        message(STATUS "Copied file directly (patch command not available)")
      endif()
    endif()
  else()
    message(STATUS "VF3 patch already applied (file exists)")
  endif()
else()
  message(WARNING "Patch file vf3_patch.patch not found")
endif()

if(GHL_MASTER_PROJECT)
  # gtest
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  include(GoogleTest)
endif()

add_library(ghl SHARED ${GHL_ALL_PURE_HEADERS})
set_target_properties(ghl PROPERTIES LINKER_LANGUAGE CXX)

# Link requested Boost libraries
foreach(boost_lib ${BOOST_INCLUDE_LIBRARIES})
  if(TARGET Boost::${boost_lib})
    target_link_libraries(ghl PUBLIC Boost::${boost_lib})
  endif()
endforeach()


add_library(ghl::ghl ALIAS ghl)
target_include_directories(ghl PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")

# cereal
if(GHL_SERIALIZATION)
  if(NOT TARGET cereal)
    CPMAddPackage(
      NAME cereal
      VERSION 1.3.2
      GITHUB_REPOSITORY USCiLab/cereal
      OPTIONS "JUST_INSTALL_CEREAL ON"
    )
  else()
    message("Found previous target cereal")
  endif()

  target_compile_definitions(ghl PUBLIC GHL_SERIALIZATION)
  target_link_libraries(ghl PUBLIC cereal::cereal)
endif()

if(GHL_BINDINGS)
  find_package(Python3 COMPONENTS Interpreter Development)
  include_directories(${Python3_INCLUDE_DIRS})
  find_library(pybind11_lib pybind11::lto)

  if(NOT TARGET pybind11_headers)
    CPMAddPackage(
      NAME pybind11
      GIT_REPOSITORY https://github.com/pybind/pybind11.git
      VERSION 2.13.6
      DOWNLOAD_ONLY
    )
    message(${pybind11_SOURCE_DIR})
    target_include_directories(ghl PUBLIC "${pybind11_SOURCE_DIR}/include")
  else()
    # I suspect this won't always work as its not guaranteed the
    # pybind11/include files will be linked to ghl
    message("Found previous target pybind11")
  endif()

  pybind11_add_module(ghl_bindings include/ghl/bindings/graph_bindings.cpp)
  target_sources(ghl PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include/ghl/bindings/graph_bindings.cpp")

  foreach(boost_lib ${BOOST_INCLUDE_LIBRARIES})
    if(TARGET Boost::${boost_lib})
      target_link_libraries(ghl_bindings PRIVATE Boost::${boost_lib})
    endif()
  endforeach()

  target_link_libraries(ghl_bindings PRIVATE ghl)

  if(APPLE)
    set_target_properties(ghl_bindings PROPERTIES LINK_FLAGS "-undefined dynamic_lookup" SUFFIX ".so")
  endif(APPLE)

  set_target_properties(ghl_bindings PROPERTIES PREFIX "" SUFFIX ".so")

  if(GHL_SERIALIZATION)
    target_sources(ghl PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include/ghl/cereal/pybind11.cpp")

    foreach(a_lib ${BOOST_INCLUDE_LIBRARIES})
      target_link_libraries(ghl PRIVATE Boost::${a_lib})
    endforeach()
  endif()

  target_compile_definitions(ghl PUBLIC GHL_BINDINGS)
  target_compile_definitions(ghl_bindings PUBLIC GHL_BINDINGS)
  add_library(ghl::bindings ALIAS ghl_bindings)
  install(TARGETS ghl_bindings DESTINATION "ghl_bindings")
endif()


if(GHL_MASTER_PROJECT)
  add_subdirectory(tests/cpp)
  add_subdirectory(examples)

  if(GHL_BINDINGS)
    add_custom_command(
      TARGET ghl_bindings POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/tests/
      COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/tests/python/
      COMMAND ${CMAKE_COMMAND} -E copy
      ${CMAKE_BINARY_DIR}/ghl_bindings.so
      ${CMAKE_BINARY_DIR}/tests/python/
      COMMENT "Copying ghl_bindings.so..."
    )

    add_custom_command(
      TARGET ghl_bindings POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_CURRENT_SOURCE_DIR}/tests
      ${CMAKE_CURRENT_BINARY_DIR}/tests
      COMMENT "Copying tests directory..."
    )
    if(DEFINED PY_BUILD_CMAKE_IMPORT_NAME)
      install(TARGETS ghl_bindings
            DESTINATION "${PY_BUILD_CMAKE_IMPORT_NAME}")
    endif()
  endif()
endif()
