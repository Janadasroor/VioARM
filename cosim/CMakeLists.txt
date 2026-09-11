# VioARM d_cosim shim build.
#
# Builds `libvioarm_cosim.so` (the `simulation=` target ngspice's d_cosim
# code model dlopens) against the `vioarm-cosim` cdylib and VioMATRIXC
# headers. Single-chip (cosim-roadmap P2-4 gates multi-instance).
#
# Usage (VioARM repo root):
#   cmake -S cosim -B cosim/build -DVioMATRIXC_ROOT=$HOME/cpp_projects/VioMATRIXC
#   cmake --build cosim/build
#
# The netlist then references it exactly like VioAVR's shim:
#   A_MCU [ ... 64 digital inputs ... ] [ ... 64 digital outputs ... ] mcu_model
#   .model mcu_model d_cosim(simulation="./cosim/build/libvioarm_cosim_shim.so"
#       sim_args=["stm32f103","fw.hex"] queue_size=1024 irreversible=1 delay=1e-9)
# NOTE: 64 pins (ports A..D), not the full 176: d_cosim instances past
# ~136 ports corrupt memory in this ngspice tree (see the shim header).
# NOTE: the shim .so is deliberately NOT named libvioarm_cosim.so — that is
# the Rust cdylib's name, and the collision makes the linker drop the dep.

cmake_minimum_required(VERSION 3.20)
project(vioarm_cosim_shim LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# VioARM workspace root (this file lives in <root>/cosim).
get_filename_component(VIOARM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

# VioMATRIXC tree: ngspice headers + generated config.h.
set(VioMATRIXC_ROOT "$ENV{HOME}/cpp_projects/VioMATRIXC"
    CACHE PATH "Path to VioMATRIXC source tree")
if(EXISTS "${VioMATRIXC_ROOT}/release/src/include/ngspice/config.h")
    set(VioMATRIXC_BUILD_INCLUDE "${VioMATRIXC_ROOT}/release/src/include")
elseif(EXISTS "${VioMATRIXC_ROOT}/build/src/include/ngspice/config.h")
    set(VioMATRIXC_BUILD_INCLUDE "${VioMATRIXC_ROOT}/build/src/include")
else()
    set(VioMATRIXC_BUILD_INCLUDE "${VioMATRIXC_ROOT}/src/include"
        CACHE PATH "VioMATRIXC build include dir (needs ngspice/config.h)")
endif()
if(NOT EXISTS "${VioMATRIXC_BUILD_INCLUDE}/ngspice/config.h")
    message(WARNING "ngspice/config.h not found under ${VioMATRIXC_BUILD_INCLUDE}; "
        "configure a VioMATRIXC build (release/ or build/) first.")
endif()

# The Rust cdylib: built here via cargo so one cmake invocation suffices.
set(VIOARM_CDYLIB "${VIOARM_ROOT}/target/release/libvioarm_cosim.so")
add_custom_command(
    OUTPUT "${VIOARM_CDYLIB}"
    COMMAND cargo build --release -p vioarm-cosim
    WORKING_DIRECTORY "${VIOARM_ROOT}"
    COMMENT "Building vioarm-cosim cdylib with cargo"
    VERBATIM
)
add_custom_target(vioarm_cosim_cdylib DEPENDS "${VIOARM_CDYLIB}")

add_library(vioarm_cosim SHARED vioarm_cosim_shim.cpp)
add_dependencies(vioarm_cosim vioarm_cosim_cdylib)
target_include_directories(vioarm_cosim PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${VioMATRIXC_ROOT}/src/include"
    "${VioMATRIXC_BUILD_INCLUDE}"
)
# Link the cdylib by full path (never -l: the names must not collide).
target_link_libraries(vioarm_cosim PRIVATE "${VIOARM_CDYLIB}")
set_target_properties(vioarm_cosim PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    OUTPUT_NAME "vioarm_cosim_shim"
    PREFIX "lib"
)
target_compile_options(vioarm_cosim PRIVATE -Wall -Wextra)

# ------------------------------------------------------------------
# vioarm.cm — VioARM XSPICE analog code-model library.
# Loaded by ngspice via .spiceinit: codemodel /path/to/vioarm.cm
#
# vioarm_adc_bridge: analog node voltage -> apin injection (ADC path).
# vioarm_dac_bridge: staged apin word -> analog node voltage (DAC path).
# vioarm_vcc_bridge: unity-gain rail mirror (brown-out hook lands here).
# Both live bridges resolve the active handle with dlsym(RTLD_DEFAULT),
# so libvioarm_cosim_shim.so must be loaded RTLD_GLOBAL (d_cosim does).
# Standalone module — no VioARM link deps (symbols resolve at run time).
# ------------------------------------------------------------------
set(VIOARM_CM_SRCS
    vioarm_analog_bridge/ifspec.c
    vioarm_analog_bridge/cfunc.c
    vioarm_analog_bridge/dlmain.c
)

add_library(vioarm_cm MODULE ${VIOARM_CM_SRCS})
target_include_directories(vioarm_cm PRIVATE
    "${VioMATRIXC_ROOT}/src/include"
    "${VioMATRIXC_BUILD_INCLUDE}"
)
target_link_libraries(vioarm_cm PRIVATE ${CMAKE_DL_LIBS})
set_target_properties(vioarm_cm PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    PREFIX ""
    OUTPUT_NAME "vioarm"
    SUFFIX ".cm"
)
target_compile_options(vioarm_cm PRIVATE -Wall -Wextra)
