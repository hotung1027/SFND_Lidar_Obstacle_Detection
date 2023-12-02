cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(/opt/homebrew/Cellar/boost/1.83.0/lib)
set(BOOST_LIBRARY_DIR /opt/homebrew/Cellar/boost/1.83.0/lib)
set(boost_root /opt/homebrew/Cellar/boost/1.83.0)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})

add_executable (quizRansac src/quiz/ransac/ransac2d.cpp src/render/render.cpp)
target_link_libraries (quizRansac ${PCL_LIBRARIES})



