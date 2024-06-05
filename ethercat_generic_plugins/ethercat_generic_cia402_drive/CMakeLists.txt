cmake_minimum_required(VERSION 3.8)
project(ethercat_generic_cia402_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
# =================

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(ethercat_interface REQUIRED)
find_package(ethercat_generic_slave REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# < End Find dependencies


file(GLOB_RECURSE PLUGINS_SRC src/*.cpp)

add_library(${PROJECT_NAME} ${PLUGINS_SRC})

target_link_libraries( ${PROJECT_NAME} PUBLIC
  ethercat_interface
  ethercat_generic_slave
  pluginlib
  yaml-cpp::yaml-cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(
#    ${PROJECT_NAME}
#    ethercat_interface
#    ethercat_generic_slave
#    pluginlib
#  )

pluginlib_export_plugin_description_file(ethercat_interface ethercat_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  ethercat_interface
  ethercat_generic_slave
  pluginlib
  yaml_cpp_vendor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Test Load EtherCAT modules
  ament_add_gmock(
    test_load_generic_plugins
    test/test_load_ec_modules.cpp
  )
  target_include_directories(test_load_generic_plugins PRIVATE include)
  # ament_target_dependencies(test_load_generic_plugins
  #   pluginlib
  #   ethercat_interface
  # )

  # Test Generic EtherCAT CIA402 Drive Plugin
  ament_add_gmock(
    test_generic_ec_cia402_drive
    test/test_generic_ec_cia402_drive.cpp
  )
  target_include_directories(test_generic_ec_cia402_drive PRIVATE include)
  target_link_libraries(test_generic_ec_cia402_drive ${PROJECT_NAME})
  
  # ament_target_dependencies(test_generic_ec_cia402_drive
  #   pluginlib
  #   ethercat_interface
  #   ethercat_generic_slave
  # )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
