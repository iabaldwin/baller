cmake_minimum_required (VERSION 3.19)
project (baller)

# Currently only MacOS supported
if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  message(FATAL_ERROR "Only MacOS supported currently")
endif()
find_library(CORE_GRAPHICS CoreGraphics)
find_library(COCOA COCOA)
find_library(IOKIT IOKit)
find_library(CORE_FOUNDATION CoreFoundation)
find_library(OPENGL opengl)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

set (CMAKE_CXX_STANDARD 20)
include_directories(/opt/local/include)
link_directories(/opt/local/lib)
include_directories(${PROJECT_SOURCE_DIR}/colormap/include)

add_executable(render render.cc)
target_include_directories(render
                          PRIVATE
                          ${EIGEN3_INCLUDE_DIR})
target_link_libraries(render
                      raylib
                      ${CORE_GRAPHICS}
                      ${CORE_FOUNDATION}
                      ${IOKIT}
                      ${COCOA}
                      ${OPENGL}
                      ${CERES_LIBRARIES})

add_executable(simple_scene simple_scene.cc)
target_include_directories(simple_scene PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(simple_scene ${CERES_LIBRARIES})
