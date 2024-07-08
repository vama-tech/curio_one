cmake_minimum_required(VERSION 3.5)
project(curio_one)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  curio_one
  SHARED
  include/curio_system.cpp

)

target_link_libraries(curio_one PUBLIC serial)
target_compile_features(curio_one PUBLIC cxx_std_17)
target_include_directories(curio_one PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:curio_one>
)
ament_target_dependencies(
  curio_one PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "CURIO_ONE_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface curio_one.xml)

install(TARGETS curio_one   
  EXPORT export_curio_one   
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME})


install(PROGRAMS scripts/range.py
       DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/1_range.py
       DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/2_range.py
       DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/icm20948_node.py
       DESTINATION lib/${PROJECT_NAME}
)
ament_package()
