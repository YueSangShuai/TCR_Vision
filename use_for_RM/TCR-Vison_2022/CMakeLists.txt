cmake_minimum_required(VERSION 3.22)
project(Toauto_aimHK)

set(CMAKE_CXX_STANDARD 20)



find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CUDA)


find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Fmt_INCLUDE_DIRS})
include_directories(${TENSORRT_PATH}/include) #加这一句
include_directories(${CUDA_INCLUDE_DIRS}) #加这一句
include_directories(${Sophus_INCLUDE_DIRS})

# 添加Eigen头文件
include_directories(${EIGEN3_INCLUDE_DIR})
#线程
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-pthread")

cuda_add_executable(Toauto_aimHK main.cpp
        Thread/MyThread.h
        Thread/MyThread.cpp
        SerialPort/SerialPort.h
        SerialPort/SerialPort.cpp
        HaikangCamera/CameraRGB/HaiKangCameraRGB.h
        HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp
        HaikangCamera/CameraGray/HaiKangCameraGray.h
        HaikangCamera/CameraGray/HaiKangCameraGray.cpp
        AutoShoot/Logger/Logger.h
        AutoShoot/TRT/TRTModule.h
        AutoShoot/TRT/TRTModule.cpp
        Analyze_data/Analyze_Data.h
        Analyze_data/Analyze_Data.cpp
        PNP_Distance/PNP_Distance.h
        PNP_Distance/PNP_Distance.cpp
        Kalman/Kalman.h
        Kalman/Kalman.cpp
        Struct.h)



#线程
target_link_libraries(Toauto_aimHK pthread)

file(GLOB TENSORRT_LIBS "${TENSORRT_LIB_PATH}/*.so")#添加这句
# 包含海康相机动态链接库
find_library(MVS_LIB libMvCameraControl.so /opt/MVS/lib/64/)
target_link_libraries(Toauto_aimHK ${MVS_LIB} ${OpenCV_LIBS} fmt::fmt yaml-cpp)

target_link_libraries(${PROJECT_NAME}  ${TENSORRT_LIBS} ${CUDA_LIBRARIES} cuda nvonnxparser nvinfer fmt ${Eigen3_LIBRARIES})

target_link_libraries(Toauto_aimHK
        Sophus::Sophus
        )

