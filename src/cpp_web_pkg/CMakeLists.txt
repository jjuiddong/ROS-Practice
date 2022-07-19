cmake_minimum_required(VERSION 3.5)
project(cpp_web_pkg)

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

set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME ON) 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(${Boost_INCLUDE_DIRS}) 

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

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

function(advanced_custom_executable target)
  add_executable(${target}_node
  src/${target}.cpp

  ./lib/common/etc/filepath1.cpp
  ./lib/common/thread/thread1.cpp 
  ./lib/common/etc/dbg.cpp 
  ./lib/common/etc/date.cpp 
  ./lib/common/thread/wqsemaphore.cpp 
  ./lib/common/thread/tpsemaphore.cpp 
  ./lib/network/session.cpp  
  ./lib/common/math/line.cpp 
  ./lib/common/math/matrix44.cpp 
  ./lib/common/math/plane.cpp 
  ./lib/common/math/plane3.cpp 
  ./lib/common/math/quaternion.cpp 
  ./lib/common/math/ray.cpp 
  ./lib/common/math/transform.cpp 
  ./lib/common/math/triangle.cpp 
  ./lib/common/math/vector2.cpp 
  ./lib/common/math/vector3.cpp 
  ./lib/common/math/vector4.cpp 
  ./lib/common/math/vectortype.cpp 
  ./lib/network/packetheader.cpp 
  ./lib/network/packetheaderascii.cpp 
  ./lib/network/packetheaderjson.cpp 
  ./lib/network/packet.cpp 
  ./lib/common/etc/recttype.cpp 
  ./lib/common/etc/stringfunc.cpp 
  ./lib/network/protocol_dispatcher.cpp 
  ./lib/network/networknode.cpp 
  ./lib/common/etc/genid.cpp 
  ./lib/network/utility/log.cpp 
  ./lib/network/utility/packetlog.cpp 
  ./lib/common/etc/date2.cpp 
  ./lib/network/socketbuffer.cpp 
  ./lib/network/utility/utility.cpp 
  ./lib/network/prtcompiler/ProtocolDefine.cpp 
  ./lib/network/packetqueue.cpp 
  ./lib/network/launcher.cpp 
  ./lib/network/tcpserver.cpp 
  ./lib/common/etc/timer.cpp 
  ./lib/network/tcpclient.cpp 
  ./lib/network/udpserver.cpp 
  ./lib/network/udpclient.cpp 
  ./lib/network/webserver.cpp 
  ./lib/network/webclient.cpp 
  ./lib/network/netcontroller.cpp 
  ./lib/network/protocol_basic_dispatcher.cpp 
  ./lib/network/protocol_all_handler.cpp 
  ./lib/network/websession.cpp  

  ./src/protocol/webrelay_Protocol.cpp
  ./src/protocol/webrelay_ProtocolHandler.cpp
  ./src/protocol/rtrs_Protocol.cpp
  ./src/protocol/rtrs_ProtocolHandler.cpp
  ./src/protocol/robot_Protocol.cpp
  ./src/protocol/robot_ProtocolHandler.cpp
  
  ./src/ctrlsvr/controlserver.cpp
  ./src/ctrlsvr/robot.cpp
  ./src/robotcli/robot_client.cpp  
  )
  
  ament_target_dependencies(${target}_node
    "rclcpp"
    "example_interfaces"
    "geometry_msgs"
    "std_msgs"
    "nav_msgs"
    "robot_interfaces")
  install(TARGETS ${target}_node
  DESTINATION lib/${PROJECT_NAME})
endfunction()

advanced_custom_executable(servermain)
advanced_custom_executable(robotmain)

target_link_libraries(servermain_node pthread PocoNet PocoUtil PocoFoundation ${Boost_LIBRARIES})
target_link_libraries(robotmain_node pthread PocoNet PocoUtil PocoFoundation)

ament_package()
