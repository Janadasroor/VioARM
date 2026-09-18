# FindVioARM.cmake — locate an installed VioARM embedding ABI (E2 packaging).
#
# Install layout expected (see `packaging/README.md`):
#   <prefix>/include/vioarm_cosim.h
#   <prefix>/lib/libvioarm_cosim.so[.1] (or .a / .dylib / .lib)
#
# Imported target: VioARM::vioarm_cosim
# Result variables: VioARM_FOUND, VioARM_INCLUDE_DIR, VioARM_LIBRARY,
#   VioARM_VERSION (from pkg-config when available, else "unknown").
#
# Hints: VioARM_ROOT (env or cache var), pkg-config module `vioarm-cosim`.

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_VIOARM QUIET vioarm-cosim)
endif()

if(DEFINED ENV{VioARM_ROOT} AND NOT DEFINED VioARM_ROOT)
    set(VioARM_ROOT "$ENV{VioARM_ROOT}")
endif()

find_path(VioARM_INCLUDE_DIR
    NAMES vioarm_cosim.h
    HINTS ${VioARM_ROOT} ${PC_VIOARM_INCLUDEDIR}
    PATH_SUFFIXES include
    DOC "vioarm_cosim.h location"
)

find_library(VioARM_LIBRARY
    NAMES vioarm_cosim
    HINTS ${VioARM_ROOT} ${PC_VIOARM_LIBDIR}
    PATH_SUFFIXES lib
    DOC "libvioarm_cosim location"
)

if(PC_VIOARM_VERSION)
    set(VioARM_VERSION "${PC_VIOARM_VERSION}")
else()
    set(VioARM_VERSION "unknown")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VioARM
    REQUIRED_VARS VioARM_LIBRARY VioARM_INCLUDE_DIR
    VERSION_VAR VioARM_VERSION
)

if(VioARM_FOUND)
    set(VioARM_INCLUDE_DIRS "${VioARM_INCLUDE_DIR}")
    set(VioARM_LIBRARIES "${VioARM_LIBRARY}")
    if(NOT TARGET VioARM::vioarm_cosim)
        add_library(VioARM::vioarm_cosim UNKNOWN IMPORTED)
        set_target_properties(VioARM::vioarm_cosim PROPERTIES
            IMPORTED_LOCATION "${VioARM_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${VioARM_INCLUDE_DIR}"
        )
    endif()
endif()

mark_as_advanced(VioARM_INCLUDE_DIR VioARM_LIBRARY)
