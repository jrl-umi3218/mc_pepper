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
find_package(mc_rtc 1.4.0 REQUIRED)

find_description_package(pepper_description)

message("PEPPER_DESCRIPTION_PATH: ${PEPPER_DESCRIPTION_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

# Pepper custom devices library
set(devices_HDR
  src/devices/Speaker.h
  src/devices/TouchSensor.h
  src/devices/VisualDisplay.h
)
set(devices_SRC
  src/devices/Speaker.cpp
  src/devices/TouchSensor.cpp
  src/devices/VisualDisplay.cpp
)

add_library(mc_pepper_devices SHARED ${devices_SRC} ${devices_HDR})
add_library(mc_pepper::devices ALIAS mc_pepper_devices)
set_target_properties(mc_pepper_devices PROPERTIES EXPORT_NAME devices)
target_include_directories(mc_pepper_devices PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mc_pepper_devices PUBLIC mc_rtc::mc_rbdyn)
install(FILES ${devices_HDR} DESTINATION include/mc_pepper/devices)
install(TARGETS mc_pepper_devices
  EXPORT mc_pepperTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Pepper custom tasks library
set(tasks_HDR
  src/tasks/CoMRelativeBodyTask.h
)
set(tasks_SRC
  src/tasks/CoMRelativeBodyTask.cpp
)
add_library(mc_pepper_tasks SHARED ${tasks_SRC} ${tasks_HDR})
add_library(mc_pepper::tasks ALIAS mc_pepper_tasks)
set_target_properties(mc_pepper_tasks PROPERTIES EXPORT_NAME tasks)
target_include_directories(mc_pepper_tasks PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mc_pepper_tasks PUBLIC mc_rtc::mc_tasks)
install(FILES ${tasks_HDR} DESTINATION include/mc_pepper/tasks)
install(TARGETS mc_pepper_tasks
  EXPORT mc_pepperTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Pepper custom constraints library
set(constraints_HDR
  src/constraints/BoundedAccelerationConstr.h
)
set(constraints_SRC
  src/constraints/BoundedAccelerationConstr.cpp
)
add_library(mc_pepper_constraints SHARED ${constraints_SRC} ${constraints_HDR})
add_library(mc_pepper::constraints ALIAS mc_pepper_constraints)
set_target_properties(mc_pepper_constraints PROPERTIES EXPORT_NAME constraints)
target_include_directories(mc_pepper_constraints PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mc_pepper_constraints PUBLIC mc_rtc::mc_solver)
install(FILES ${constraints_HDR} DESTINATION include/mc_pepper/constraints)
install(TARGETS mc_pepper_constraints
  EXPORT mc_pepperTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Pepper robot module library
add_robot(pepper src/pepper.cpp src/pepper.h)
target_include_directories(pepper PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_link_libraries(pepper PRIVATE mc_pepper::tasks)
target_link_libraries(pepper PRIVATE mc_pepper::devices)
target_link_libraries(pepper PRIVATE mc_pepper::constraints)

install(EXPORT mc_pepperTargets
  FILE mc_pepperTargets.cmake
  NAMESPACE mc_pepper::
  DESTINATION lib/cmake/mc_pepper)
