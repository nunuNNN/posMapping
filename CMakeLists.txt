cmake_minimum_required(VERSION 2.8.3)
project(posMapping)

ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

if(YAMLCPP_STATIC_LIBRARY)
    set(YAMLCPP_STATIC libyaml-cpp.a)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)


# 添加禾赛激光雷达的驱动
add_subdirectory(library/HesaiLidar_General_SDK)

# 添加多模后处理软件库
link_directories(library/navBoxPos)

include_directories(
  ${YAML_CPP_INCLUDE_DIR}
	library/HesaiLidar_General_SDK/include  # hesai driver
	library/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
  library/navBoxDriver                    # navbox driver
  library/navBoxPos                       # navbox pos
  ${EIGEN3_INCLUDE_DIR}                   # lio&mapping
  ${PCL_INCLUDE_DIRS}
  include
)
  
add_executable(posMapping 
  src/main.cpp
  library/navBoxDriver/data_translation.cpp 
  src/slam_manager.cpp
  src/imu_process.cpp
  src/navbox_lio.cpp
  include/ikd-Tree/ikd_Tree.cpp 
)

target_link_libraries(posMapping 
  yaml-cpp
  ${PCL_LIBRARIES}
  PandarGeneralSDK
  navboxPos
)
