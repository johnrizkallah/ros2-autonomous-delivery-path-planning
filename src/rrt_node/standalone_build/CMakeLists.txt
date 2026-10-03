cmake_minimum_required(VERSION 3.16)
project(rrt_star_kd_standalone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(FLANN REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(LZ4 REQUIRED liblz4)

add_executable(rrt_star_kd_planner
  ../src/planners/rrt_star_kd_planner.cpp
)

target_include_directories(rrt_star_kd_planner PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${LZ4_INCLUDE_DIRS}
  ${FLANN_INCLUDE_DIRS}
)

target_link_directories(rrt_star_kd_planner PRIVATE
  ${JSONCPP_LIBRARY_DIRS}
  ${LZ4_LIBRARY_DIRS}
)

target_compile_options(rrt_star_kd_planner PRIVATE
  ${JSONCPP_CFLAGS_OTHER}
  ${LZ4_CFLAGS_OTHER}
)

target_link_libraries(rrt_star_kd_planner PRIVATE
  ${OpenCV_LIBS}
  yaml-cpp
  ${JSONCPP_LIBRARIES}
  ${FLANN_LIBRARIES}
  ${LZ4_LIBRARIES}
)
