cmake_minimum_required(VERSION 3.15)
project(sgt)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
add_executable(sgt main.cpp slam.cpp slam.h mapper.cpp mapper.h pose.cpp pose.h frame_snapshot.h tile.cpp tile.h)

include(FindGDAL)
include(FindTIFF)

find_package(Python3 COMPONENTS Development NumPy)

#find_library(gdal NAMES gdal)
target_include_directories(sgt PRIVATE
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIRS}
        ${TIFF_INCLUDE_DIRS}
        )
target_link_libraries(sgt
        ${OpenCV_LIBS}
        Python3::Python
        Python3::NumPy
        ${GDAL_LIBRARY}
        ${TIFF_LIBRARY}
        )