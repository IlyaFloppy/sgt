cmake_minimum_required(VERSION 3.15)
project(sgt)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
add_executable(sgt main.cpp slam.cpp slam.h)

find_package(Python3 COMPONENTS Development NumPy)
target_include_directories(sgt PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_link_libraries(sgt ${OpenCV_LIBS} Python3::Python Python3::NumPy)