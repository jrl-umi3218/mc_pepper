cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost REQUIRED COMPONENTS timer serialization filesystem)
find_package(pepper_description REQUIRED)
find_package(mc_rtc REQUIRED)

if("${pepper_description_INSTALL_PREFIX}" STREQUAL "")
  if("${pepper_description_SOURCE_PREFIX}" STREQUAL "")
    message(FATAL_ERROR "Your pepper_description package does not define where to find the data")
  endif()
  set(PEPPER_DESCRIPTION_PATH "${pepper_description_SOURCE_PREFIX}")
else()
  set(PEPPER_DESCRIPTION_PATH "${pepper_description_INSTALL_PREFIX}/share/pepper_description")
endif()

message("PEPPER_DESCRIPTION_PATH: ${PEPPER_DESCRIPTION_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_robot(pepper src/pepper.cpp src/pepper.h)
target_include_directories(pepper PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
endif()
