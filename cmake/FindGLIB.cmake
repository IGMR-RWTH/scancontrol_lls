# FindGLIB.cmake
# Locate the GLib library

find_package(PkgConfig REQUIRED)
include(FindPackageHandleStandardArgs)
pkg_check_modules(GLIB REQUIRED glib-2.0)
find_package_handle_standard_args(GLIB DEFAULT_MSG)

if(GLIB_FOUND)
    message(STATUS "Found GLib: ${GLIB_VERSION}")

    # Include directories
    include_directories(${GLIB_INCLUDE_DIRS})

    # Set variables for use elsewhere
    set(GLIB_LIBRARIES ${GLIB_LIBRARIES} CACHE INTERNAL "GLib libraries")
    set(GLIB_INCLUDE_DIRS ${GLIB_INCLUDE_DIRS} CACHE INTERNAL "GLib include directories")
else()
    message(FATAL_ERROR "Could not find GLib library.")
endif()