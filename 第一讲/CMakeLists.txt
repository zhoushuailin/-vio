# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 声明一个 cmake 工程
project(ex1)

# 设置编译模式
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 ${SSE_FLAGS} -g -O3 -march=native")

#寻找包
#find_package(OpenCV 4 REQUIRED)
#find_package(eigen3 REQUIRED)
 
#包含头文件
#include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${PROJECT_SOURCE_DIR}/include)
#include_directories(${Eigen3_INCLUDE_DIR})
#message("Eigen found(include:${Eigen_INCLUDE_DIR})" ) 
include_directories( "/usr/include/eigen3" )### ???疑问，用find_package的方法就会报错
include_directories( "/usr/include/Sophus" )
 
#添加可执行文件
add_executable(ex1 ex1.cpp)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
#将库文件链接到可执行程序上
#target_link_libraries(yolo-dnn ${OpenCV_LIBS})
