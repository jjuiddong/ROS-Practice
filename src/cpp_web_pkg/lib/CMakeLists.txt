cmake_minimum_required(VERSION 3.5)
project(netlib)

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


function(advanced_custom_executable target)
  add_executable(${target}
  ${target}.cpp

  ./common/etc/filepath1.cpp
  ./common/thread/thread1.cpp 
  ./common/etc/dbg.cpp 
  ./common/etc/date.cpp 
  ./common/thread/wqsemaphore.cpp 
  ./common/thread/tpsemaphore.cpp 
  ./network/session.cpp  
  ./common/math/line.cpp 
  ./common/math/matrix44.cpp 
  ./common/math/plane.cpp 
  ./common/math/plane3.cpp 
  ./common/math/quaternion.cpp 
  ./common/math/ray.cpp 
  ./common/math/transform.cpp 
  ./common/math/triangle.cpp 
  ./common/math/vector2.cpp 
  ./common/math/vector3.cpp 
  ./common/math/vector4.cpp 
  ./common/math/vectortype.cpp 
  ./network/packetheader.cpp 
  ./network/packetheaderascii.cpp 
  ./network/packet.cpp 
  ./common/etc/recttype.cpp 
  ./common/etc/stringfunc.cpp 
  ./network/protocol_dispatcher.cpp 
  ./network/networknode.cpp 
  ./common/etc/genid.cpp 
  ./network/utility/log.cpp 
  ./network/utility/packetlog.cpp 
  ./common/etc/date2.cpp 
  ./network/socketbuffer.cpp 
  ./network/utility/utility.cpp 
  ./network/prtcompiler/ProtocolDefine.cpp 
  ./network/packetqueue.cpp 
  ./network/launcher.cpp 
  ./network/tcpserver.cpp 
  ./common/etc/timer.cpp 
  ./network/tcpclient.cpp 
  ./network/udpserver.cpp 
  ./network/udpclient.cpp 
  ./network/webserver.cpp 
  ./network/webclient.cpp 
  ./network/netcontroller.cpp 
  ./network/protocol_basic_dispatcher.cpp 
  ./network/protocol_all_handler.cpp 
  ./network/websession.cpp  
  )
  
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

advanced_custom_executable(main)

target_link_libraries(main pthread PocoNet PocoUtil PocoFoundation)


