cmake_minimum_required(VERSION 3.1)

project(mc_pepper LANGUAGES CXX VERSION 1.0.0)

enable_testing()
set(CMAKE_CXX_STANDARD 11)

include(CMakePackageConfigHelpers)
set(VERSION_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(PROJECT_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
set(CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}")
write_basic_package_version_file(
  ${VERSION_CONFIG}
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in" "${PROJECT_CONFIG}" INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION "${CONFIG_INSTALL_DIR}")

find_package(Boost REQUIRED COMPONENTS timer serialization filesystem)
find_package(mc_rtc REQUIRED)

find_description_package(pepper_description)

message("PEPPER_DESCRIPTION_PATH: ${PEPPER_DESCRIPTION_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

set(sensors_HDR
  src/sensors/Speaker.h
  src/sensors/TouchSensor.h
  src/sensors/VisualDisplay.h
)
set(sensors_SRC
  src/sensors/Speaker.cpp
  src/sensors/TouchSensor.cpp
  src/sensors/VisualDisplay.cpp
)

add_library(mc_pepper_sensors SHARED ${sensors_SRC} ${sensors_HDR})
add_library(mc_pepper::sensors ALIAS mc_pepper_sensors)
set_target_properties(mc_pepper_sensors PROPERTIES EXPORT_NAME sensors)
target_include_directories(mc_pepper_sensors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mc_pepper_sensors PUBLIC mc_rtc::mc_rbdyn)
install(FILES ${sensors_HDR} DESTINATION include/mc_pepper/sensors)
install(TARGETS mc_pepper_sensors
  EXPORT mc_pepperTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_robot(pepper src/pepper.cpp src/pepper.h)
target_include_directories(pepper PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_link_libraries(pepper PRIVATE mc_pepper::sensors)

install(EXPORT mc_pepperTargets
  FILE mc_pepperTargets.cmake
  NAMESPACE mc_pepper::
  DESTINATION lib/cmake/mc_pepper)
