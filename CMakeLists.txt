cmake_minimum_required( VERSION 3.16.3 )

project( arx5-sdk )

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

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(
    $ENV{CONDA_PREFIX}/include/
    $ENV{CONDA_PREFIX}/include/urdfdom_headers
)

add_library(ArxJointController SHARED
    src/app/joint_controller.cpp
    src/utils.cpp
)
target_link_libraries(ArxJointController
    ${LIB_DIR}/libhardware.so
    ${LIB_DIR}/libsolver.so
    Eigen3::Eigen
    Threads::Threads
    spdlog::spdlog
    kdl_parser
    orocos-kdl
)
# add_library(ArxHighlevelController SHARED
#     src/app/high_level.cpp
#     src/utils.cpp
# )
# target_link_libraries(ArxHighlevelController
#     ${LIB_DIR}/libhardware.so
#     ${LIB_DIR}/libsolver.so
#     Eigen3::Eigen
#     Threads::Threads
#     spdlog::spdlog
#     kdl_parser
#     orocos-kdl
# )

# Hack for py310 (conda environment is slightly different from other python versions)
target_include_directories(ArxJointController PUBLIC $ENV{CONDA_PREFIX}/include/kdl_parser  $ENV{CONDA_PREFIX}/include/urdfdom_headers)
# target_include_directories(ArxHighlevelController PUBLIC $ENV{CONDA_PREFIX}/include/kdl_parser  $ENV{CONDA_PREFIX}/include/urdfdom_headers)


add_executable(test_joint_controller examples/test_joint_controller.cpp)
target_link_libraries(test_joint_controller 
    ${LIB_DIR}/libhardware.so 
    ${LIB_DIR}/libsolver.so 
    ArxJointController
    Eigen3::Eigen
    Threads::Threads
    spdlog::spdlog
    kdl_parser
    orocos-kdl
)

# add_executable(test_high_level examples/test_high_level.cpp)
# target_link_libraries(test_high_level 
#     ${LIB_DIR}/libhardware.so 
#     ${LIB_DIR}/libsolver.so
#     ArxHighlevelController
#     spdlog::spdlog
#     Eigen3::Eigen
#     Threads::Threads
#     kdl_parser
#     orocos-kdl
# )

add_subdirectory(python)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
)

install(FILES ${LIB_DIR}/libhardware.so ${LIB_DIR}/libsolver.so
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
)

# install(TARGETS ArxJointController ArxHighlevelController
#     LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
# )
