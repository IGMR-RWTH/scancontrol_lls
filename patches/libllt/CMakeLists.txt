cmake_minimum_required(VERSION 3.10)

project(libllt
    VERSION 1.0.1
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(ament_cmake REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake")

# Required libraries
find_package(ARAVIS REQUIRED)
find_package(GLIB REQUIRED)
find_package(libmescan REQUIRED)


add_library(llt SHARED src/llt.cpp)

target_include_directories(llt
    PUBLIC
     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)


target_link_libraries(llt
    PUBLIC
        libmescan::mescan
        ${ARAVIS_LIBRARIES}
        ${GLIB_LIBRARIES}
)


set_target_properties(llt PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER include/llt.h
)


include(GNUInstallDirs)

install(TARGETS llt
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    libmescan
    ARAVIS
    GLIB)

    ament_package()
