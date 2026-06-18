cmake_minimum_required(VERSION 3.10)

project(libmescan
    VERSION 1.0.1
    LANGUAGES C
)

find_package(ament_cmake REQUIRED)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake")


find_package(ARAVIS REQUIRED)
find_package(GLIB REQUIRED)


set(HEADERS
    include/mescan.h
    include/mescan_basic.h
    include/mescan_adv.h
    include/LLTDataTypes.h
)

set(SOURCES
    src/mescan_basic.c
    src/mescan_adv.c
)

add_library(mescan SHARED ${SOURCES})

target_include_directories(mescan
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${ARAVIS_INCLUDE_DIRS}
    ${GLIB_INCLUDE_DIRS}
)

target_link_libraries(mescan
    PUBLIC
    ${ARAVIS_LIBRARIES}
    ${GLIB_LIBRARIES}
)

set_target_properties(mescan PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER "${HEADERS}"
)

include(GNUInstallDirs)

install(TARGETS mescan
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
    ARAVIS
    GLIB
)
ament_package()

