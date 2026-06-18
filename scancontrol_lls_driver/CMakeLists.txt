cmake_minimum_required(VERSION 3.18)
project(scancontrol_lls_driver)
set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(scancontrol_lls_msgs REQUIRED)
find_package(libllt REQUIRED)
find_package(libmescan REQUIRED)

set(DEPENDENCIES
  rclcpp
  std_srvs
  sensor_msgs
  scancontrol_lls_msgs
  libllt
  libmescan
)

add_library(lls_driver SHARED src/scanner_interface.cpp src/lls_driver.cpp src/message_point.cpp src/buffer.cpp src/profile_publisher.cpp)
set_target_properties(lls_driver PROPERTIES CXX_STANDARD 20)

target_include_directories(lls_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lls_driver ${DEPENDENCIES})

add_executable(lls_node src/main.cpp)
target_link_libraries(lls_node PRIVATE lls_driver)

install(TARGETS lls_driver
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

install(TARGETS lls_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    ${PROJECT_SOURCE_DIR}/ext/me_sdk/examples/*/*/*
    ${PROJECT_SOURCE_DIR}/ext/me_sdk/libllt/*/*
    ${PROJECT_SOURCE_DIR}/ext/me_sdk/libmescan/*/*
    ${PROJECT_SOURCE_DIR}/ext/me_sdk/misc/*
    ${PROJECT_SOURCE_DIR}/ext/me_sdk/python_bindings/pylinllt/*)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # disable flake8 linter, since we do not have python code here
  set(ament_cmake_flake8_FOUND TRUE)
  # disable flake8 linter, since we do not have python code here
  set(ament_cmake_pep257_FOUND TRUE)


  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(${PROJECT_NAME}_test test/profile_converter_test.cpp test/buffer_test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  # Add dependencies for the C++ test executable
  ament_target_dependencies(${PROJECT_NAME}_test
    ${DEPENDENCIES}
  )
  target_link_libraries(${PROJECT_NAME}_test lls_driver)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
