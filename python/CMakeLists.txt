cmake_minimum_required( VERSION 3.16.3 )


find_package(pybind11 REQUIRED PATHS ${PYBIND_PATH}/pybind11/share/cmake/pybind11 NO_DEFAULT_PATH)

pybind11_add_module(arx5_interface 
arx5_pybind.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../src/app/low_level.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../src/app/high_level.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../src/utils.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/libcan)
add_compile_options(-pthread)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lib/aarch64)

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lib/x86_64)
endif()

target_link_libraries(arx5_interface PRIVATE 
${LIB_DIR}/libhardware.so
${LIB_DIR}/libsolver.so
/usr/local/lib/libkdl_parser.so
orocos-kdl
pthread
)

# # Optional: set the output directory for the built module
set_target_properties(arx5_interface PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(arx5_interface PROPERTIES PREFIX "")
set_target_properties(arx5_interface PROPERTIES OUTPUT_NAME "arx5_interface")
