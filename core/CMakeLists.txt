cmake_minimum_required(VERSION 3.25)
project(A320Core VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Unreal links against the release DLL CRT in every configuration.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")

option(A320_BUILD_TESTS "Build the headless tests" ON)
set(A320_JSBSIM_SOURCE_DIR "" CACHE PATH "Local JSBSim checkout (fetched from GitHub when empty)")
set(A320_JSBSIM_TAG "v1.3.1" CACHE STRING "JSBSim git tag to fetch")

# JSBSim: static library only, none of its tools, bindings or docs.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_JULIA_PACKAGE OFF CACHE BOOL "" FORCE)
set(BUILD_MATLAB_SFUNCTION OFF CACHE BOOL "" FORCE)

if(A320_JSBSIM_SOURCE_DIR)
  add_subdirectory(${A320_JSBSIM_SOURCE_DIR} jsbsim EXCLUDE_FROM_ALL SYSTEM)
else()
  include(FetchContent)
  if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
  endif()
  FetchContent_Declare(jsbsim
    GIT_REPOSITORY https://github.com/JSBSim-Team/jsbsim.git
    GIT_TAG ${A320_JSBSIM_TAG}
    GIT_SHALLOW TRUE)
  FetchContent_GetProperties(jsbsim)
  if(NOT jsbsim_POPULATED)
    FetchContent_Populate(jsbsim)
    add_subdirectory(${jsbsim_SOURCE_DIR} ${jsbsim_BINARY_DIR} EXCLUDE_FROM_ALL SYSTEM)
  endif()
endif()

set(A320_WARNINGS $<IF:$<CXX_COMPILER_ID:MSVC>,/W4,-Wall -Wextra -Wpedantic -Wshadow>)

add_library(a320core_static STATIC
  src/Airport.cpp
  src/Audio.cpp
  src/Autopilot.cpp
  src/FlyByWire.cpp
  src/Geo.cpp
  src/Ils.cpp
  src/SimClock.cpp
  src/Simulation.cpp
  src/Systems.cpp)
target_include_directories(a320core_static PUBLIC include)
target_compile_definitions(a320core_static PUBLIC A320CORE_STATIC)
target_compile_options(a320core_static PRIVATE ${A320_WARNINGS})
target_link_libraries(a320core_static PRIVATE libJSBSim)

# The DLL Unreal loads: JSBSim and the core behind a plain C API.
add_library(A320Core SHARED src/CApi.cpp)
target_include_directories(A320Core PUBLIC include)
target_compile_definitions(A320Core PRIVATE A320CORE_BUILD)
target_compile_options(A320Core PRIVATE ${A320_WARNINGS})
target_link_libraries(A320Core PRIVATE a320core_static libJSBSim)
set_target_properties(A320Core PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # Keep JSBSim's bundled expat symbols out of the DLL's export table.
  target_link_options(A320Core PRIVATE -Wl,--exclude-libs,ALL)
endif()

install(TARGETS A320Core RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
install(FILES include/a320/a320_api.h include/a320/Geometry2D.h DESTINATION include/a320)

if(A320_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
