cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)
# include_directories(/usr/lib/x86_64-linux-gnu/)
# include_directories(/usr/local/)
# link_directories(/usr/local/lib/x86_64-linux-gnu)
# link_directories(/usr/local/lib)

>>>>>>> Stashed changes
project(playback)

find_package(PCL 1.2...<1.12 REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS system filesystem REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#[[ include_directories(/opt/homebrew/Cellar/boost/1.83.0/lib)
set(BOOST_LIBRARY_DIR /opt/homebrew/Cellar/boost/1.83.0/lib)
set(boost_root /opt/homebrew/Cellar/boost/1.83.0) ]]
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})

# add_executable (quizRansac src/quiz/ransac/ransac2d.cpp src/render/render.cpp)
# target_link_libraries (quizRansac ${PCL_LIBRARIES})
#
# add_executable (quizCluster src/quiz/cluster/cluster.cpp src/render/render.cpp)
# target_link_libraries (quizCluster ${PCL_LIBRARIES})
<<<<<<< Updated upstream


