cmake_minimum_required(VERSION 3.23)
project(HaikangCamera)

set(CMAKE_CXX_STANDARD 20)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Fmt_INCLUDE_DIRS})

add_executable(HaikangCamera main.cpp
        HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp HaikangCamera/CameraRGB/HaiKangCameraRGB.h
        HaikangCamera/CameraGray/HaiKangCameraGray.cpp HaikangCamera/CameraGray/HaiKangCameraGray.h
        )



# 包含海康相机动态链接库
find_library(MVS_LIB libMvCameraControl.so /opt/MVS/lib/64/)
target_link_libraries(HaikangCamera ${MVS_LIB} ${OpenCV_LIBS} fmt::fmt yaml-cpp)