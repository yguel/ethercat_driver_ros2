cmake_minimum_required(VERSION 3.8)
project(ethercat_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ethercat_interface REQUIRED)

file(GLOB_RECURSE PLUGINS_SRC src/*.cpp)

add_library(${PROJECT_NAME} ${PLUGINS_SRC})

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ethercat_interface
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ETHERCAT_DRIVER_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface ethercat_driver_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ethercat_interface REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Ensure test data is copied to the build directory
  set(CFG_SRC ${CMAKE_CURRENT_SOURCE_DIR}/examples/configurations/safety.ros2_control.xacro)
  set(CFG_DST ${CMAKE_CURRENT_BINARY_DIR}/safety.ros2_control.xacro)

  # Add a custom command to copy the file
  add_custom_command(
    OUTPUT ${CFG_DST}
    COMMAND ${CMAKE_COMMAND} -E copy ${CFG_SRC} ${CFG_DST}
    DEPENDS ${CFG_SRC}
    COMMENT "Copying ${CFG_SRC} to ${CFG_DST}"
  )

  # Create a custom target that depends on the custom command
  add_custom_target(copy_safety_config ALL DEPENDS ${CFG_DST})

  ament_add_gmock(test_ethercat_safety_driver test/test_ethercat_safety_driver.cpp)
  target_include_directories(test_ethercat_safety_driver PRIVATE include)

  target_compile_definitions(test_ethercat_safety_driver PRIVATE "TEST_RESOURCES_DIRECTORY=\"${CMAKE_CURRENT_BINARY_DIR}\"")

  target_link_libraries(test_ethercat_safety_driver
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_ethercat_safety_driver
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    ethercat_interface
  )
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ethercat_interface
)
ament_package()
