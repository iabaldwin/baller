cmake_minimum_required (VERSION 3.19)
project (baller)

find_library(CORE_GRAPHICS CoreGraphics)
find_library(COCOA COCOA)
find_library(IOKIT IOKit)
find_library(CORE_FOUNDATION CoreFoundation)
find_library(OPENGL opengl)

find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

set (CMAKE_CXX_STANDARD 17)
include_directories(/opt/local/include)
link_directories(/opt/local/lib)

add_executable(render render.cc)
target_include_directories(render
                          PRIVATE 
                          ${OpenCV_INCLUDE_DIRS}
                          ${EIGEN3_INCLUDE_DIR}
                          ${Boost_INCLUDE_DIRS})
target_link_libraries(render 
                      raylib 
                      ${CORE_GRAPHICS} 
                      ${CORE_FOUNDATION} 
                      ${IOKIT} 
                      ${COCOA} 
                      ${OPENGL} 
                      ${Boost_LIBRARIES}
                      ${CERES_LIBRARIES}
                      ${OpenCV_LIBRARIES})

add_executable(simple_scene simple_scene.cc)
target_include_directories(simple_scene PUBLIC ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(simple_scene ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})
