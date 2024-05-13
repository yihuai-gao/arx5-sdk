cmake_minimum_required( VERSION 3.16.3 )

project( arx5_sdk )

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # global PIC

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)

set(ARX_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DARX_DIRECTORY="${ARX_DIRECTORY}") # TODO: remove this

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
endif()

set(cpp_source
    src/app/low_level.cpp
    src/utils.cpp
)
add_executable(arx5_low_level examples/test_low_level.cpp ${cpp_source})

target_link_libraries(arx5_low_level 
    ${LIB_DIR}/libhardware.so 
    Eigen3::Eigen
    Threads::Threads
    spdlog::spdlog
)
target_include_directories(arx5_low_level PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(arx5_low_level PUBLIC ${EIGEN3_INCLUDE_DIRS})



add_executable(arx5_high_level examples/test_high_level.cpp 
    ${cpp_source}
    src/app/high_level.cpp
)  
target_link_libraries(arx5_high_level 
    ${LIB_DIR}/libhardware.so 
    ${LIB_DIR}/libsolver.so 
    spdlog::spdlog
    Eigen3::Eigen
    Threads::Threads
    kdl_parser
    orocos-kdl
    pthread
)
target_include_directories(arx5_high_level PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(arx5_high_level PUBLIC ${EIGEN3_INCLUDE_DIRS})
# Hack for py310
target_include_directories(arx5_high_level PUBLIC /home/unitree/miniforge3/envs/arx-py310/include/kdl_parser  /home/unitree/miniforge3/envs/arx-py310/include/urdfdom_headers)

add_subdirectory(python)