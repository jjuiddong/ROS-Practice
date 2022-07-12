cmake_minimum_required(VERSION 3.5)
project(simple_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
  -fpermissive 
  -Wno-unused-variable 
  -Wno-unused-parameter 
  -Wno-unused-function
  -Wno-conversion-null
  -Wno-sign-compare
  -Wno-pointer-arith
  -Wno-pedantic
  -Wno-reorder
  -pthread 
  -Wno-delete-non-virtual-dtor 
  -Wno-switch 
  -Wno-format 
  -Wno-deprecated-copy
  -Wno-class-memaccess
  -Wno-implicit-fallthrough
  -Wno-catch-value
  -Wno-unused-but-set-variable  
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
  DIRECTORY config launch rviz src urdf world
  DESTINATION share/${PROJECT_NAME}
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


function(advanced_custom_executable target)
  add_executable(${target}
  src/${target}.cpp

  )
  
  ament_target_dependencies(${target}
    "rclcpp"
    "geometry_msgs"
  )
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

advanced_custom_executable(robot)

target_link_libraries(robot pthread)

ament_package()

