# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/lilybot/esp/esp-idf/components/bootloader/subproject"
  "/home/lilybot/firmware_project/micro-ROS/build/bootloader"
  "/home/lilybot/firmware_project/micro-ROS/build/bootloader-prefix"
  "/home/lilybot/firmware_project/micro-ROS/build/bootloader-prefix/tmp"
  "/home/lilybot/firmware_project/micro-ROS/build/bootloader-prefix/src/bootloader-stamp"
  "/home/lilybot/firmware_project/micro-ROS/build/bootloader-prefix/src"
  "/home/lilybot/firmware_project/micro-ROS/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/lilybot/firmware_project/micro-ROS/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/lilybot/firmware_project/micro-ROS/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
