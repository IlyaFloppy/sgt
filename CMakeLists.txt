cmake_minimum_required(VERSION 3.15)
project(sgt)

set(CMAKE_CXX_STANDARD 17)

find_package(Python3 COMPONENTS Development NumPy)
find_package(PythonLibs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
message("Protobuf_LIBRARIES: " ${Protobuf_LIBRARIES})

add_executable(
        sgt
        main.cpp
        slam.cpp
        slam.h
        tiles_storage.h
        mapper.cpp
        mapper.h
        pose.cpp
        pose.h
        frame_snapshot.h
        tile.cpp
        tile.h
        helper.h
        helper.cpp
        tiles_storage.cpp
        grpc_video_capture.h
        grpc_video_capture.cpp
        protos/video.pb.h
        protos/video.pb.cc
        protos/video.grpc.pb.h
        protos/video.grpc.pb.cc
)

include(FindGDAL)
include(FindTIFF)

include_directories(${PYTHON_INCLUDE_DIRS})

include(FetchContent)

FetchContent_Declare(
        gRPC
        GIT_REPOSITORY https://github.com/grpc/grpc
        GIT_TAG v1.32.0
)
set(FETCHCONTENT_QUIET OFF)
FetchContent_MakeAvailable(gRPC)

#find_library(gdal NAMES gdal)
target_include_directories(sgt PRIVATE
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIRS}
        ${TIFF_INCLUDE_DIRS}
        )
target_link_libraries(sgt
        ${OpenCV_LIBS}
        ${Python}
        ${NumPy}
        ${PYTHON_LIBRARIES}
        ${GDAL_LIBRARY}
        ${TIFF_LIBRARY}
        ${Protobuf_LIBRARIES}
        grpc++
        )