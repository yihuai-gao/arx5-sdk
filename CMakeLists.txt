#cmake_minimum_required( VERSION 2.8 )
cmake_minimum_required( VERSION 3.16.3 )

project( arx5_python_sdk )

set(CMAKE_BUILD_TYPE Debug)

set(cpp_source
    src/hardware/motor.cpp 
    src/app/low_level.cpp
    src/hardware/math_ops.cpp
    src/utils.cpp
)

set(high_level_src
    src/app/high_level.cpp
    )

set(ARX_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DARX_DIRECTORY="${ARX_DIRECTORY}")

add_executable(arx5_low_level examples/test_low_level.cpp ${cpp_source})

add_executable(calibrate_gripper examples/calibrate_gripper.cpp ${cpp_source})


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)#include文件夹下的头文件
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)#src文件夹下的头文件
include_directories("/usr/include/eigen3")#Eigen的头文件

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_subdirectory(python)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
endif()
target_link_libraries(arx5_low_level 
    ${LIB_DIR}/hardware/can.a 
    )
target_link_libraries(calibrate_gripper 
    ${LIB_DIR}/hardware/can.a 
    )

add_executable(arx5_high_level examples/test_high_level.cpp ${cpp_source} ${high_level_src})  
target_link_libraries(arx5_high_level 
${LIB_DIR}/hardware/can.a 
${LIB_DIR}/app/solver.a
/usr/local/lib/libkdl_parser.so
orocos-kdl
pthread
)

set(CPACK_GENERATOR DEB)
set(CPACK_PACKAGE_NAME "ARX_PYTHON")
set(CPACK_PACKAGE_VERSION "0.0.1")
set(CPACK_PACKAGE_CONTACT "yihuai@stanford.edu")
set(CMAKE_BUILD_TYPE Debug)
include(CPack)