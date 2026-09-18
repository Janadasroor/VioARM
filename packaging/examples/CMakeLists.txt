# Builds `host_minimal` against an installed (or in-tree) VioARM ABI
# (E2 packaging smoke test for FindVioARM.cmake).
cmake_minimum_required(VERSION 3.16)
project(vioarm_host_minimal C)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/..")
find_package(VioARM REQUIRED)
add_executable(host_minimal host_minimal.c)
target_link_libraries(host_minimal PRIVATE VioARM::vioarm_cosim)
