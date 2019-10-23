cmake_minimum_required(VERSION 3.1)
project(color_seg)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)


# Set opencv directory
find_package(OpenCV REQUIRED)
# find_package(Boost REQUIRED)

# include_directories(${Boost_INCLUDE_DIRS})

set(SOURCE color_seg.cpp)
add_executable(color_seg color_seg.cpp)
add_executable(main main.cpp)
target_link_libraries(color_seg ${OpenCV_LIBS}) # ${Boost_LIBRARIES})
target_link_libraries(main ${OpenCV_LIBS}) # ${Boost_LIBRARIES})

