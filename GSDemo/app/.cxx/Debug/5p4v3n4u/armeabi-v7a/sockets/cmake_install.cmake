# Install script for directory: /home/konglobemeralt/Documents/gitz/util/C/sockets

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/home/konglobemeralt/Android/Sdk/ndk/21.4.7075529/toolchains/llvm/prebuilt/linux-x86_64/bin/arm-linux-androideabi-objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  MESSAGE(STATUS "Installing target TCPUDPSocket")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/konglobemeralt/Documents/gitz/Android-GSDemo-GoogleMap/GSDemo/app/build/intermediates/cxx/Debug/5p4v3n4u/obj/armeabi-v7a/libTCPUDPSocket.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libTCPUDPSocket.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libTCPUDPSocket.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/home/konglobemeralt/Android/Sdk/ndk/21.4.7075529/toolchains/llvm/prebuilt/linux-x86_64/bin/arm-linux-androideabi-strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libTCPUDPSocket.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "/home/konglobemeralt/Documents/gitz/util/C/sockets/canhandler.hpp"
    "/home/konglobemeralt/Documents/gitz/util/C/sockets/socket.hpp"
    "/home/konglobemeralt/Documents/gitz/util/C/sockets/server.hpp"
    "/home/konglobemeralt/Documents/gitz/util/C/sockets/client.hpp"
    "/home/konglobemeralt/Documents/gitz/util/C/sockets/socketexceptions.hpp"
    "/home/konglobemeralt/Documents/gitz/util/C/sockets/tcphandler.hpp"
    "/home/konglobemeralt/Documents/gitz/util/C/sockets/udphandler.hpp"
    )
endif()

