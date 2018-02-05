cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME mc_pepper)
set(PROJECT_DESCRIPTION "Pepper RobotModule for mc_rtc")
set(PROJECT_URL https://gite.lirmm.fr/multi-contact/mc_pepper)

setup_project()

# Enable C++11
if(NOT WIN32)
  add_definitions(-std=c++0x)
endif()

# Use jrl-cmakemoduels to get mc_rtc depency
set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir")
add_required_dependency(mc_rtc)

# Find description package
macro(find_description_package desc_pkg)
  string(TOUPPER "${desc_pkg}" "DESC_PKG")
  if(NOT DEFINED ${DESC_PKG}_PATH)
    add_optional_dependency("${desc_pkg}")
    if(${${DESC_PKG}_FOUND})
      execute_process(
        COMMAND "${PKG_CONFIG_EXECUTABLE}"
        "--variable=datadir" "${desc_pkg}"
        OUTPUT_VARIABLE "${DESC_PKG}_PATH"
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    else()
      MESSAGE(WARNING "${DESC_PKG}_PATH is not defined and pkg-config failed to find ${desc_pkg}, the related robots' modules will not work properly. Check your system configuration or define ${DESC_PKG} to fix this issue.")
    endif()
  endif()
  if(DEFINED ${DESC_PKG}_PATH)
    MESSAGE(STATUS "Found ${desc_pkg}: ${${DESC_PKG}_PATH}")
  endif()
endmacro()
find_description_package(pepper_description)

set(MC_ROBOTS_INSTALL_PREFIX ${MC_RTC_PREFIX}/lib/mc_robots)

add_library(pepper SHARED src/pepper.cpp src/pepper.h)
set_target_properties(pepper PROPERTIES COMPILE_FLAGS "-DMC_ROBOTS_EXPORTS -DPEPPER_DESCRIPTION_PATH_IN=\\\"${PEPPER_DESCRIPTION_PATH}\\\"" PREFIX "")
pkg_config_use_dependency(pepper mc_rtc)
install(TARGETS pepper DESTINATION ${MC_ROBOTS_INSTALL_PREFIX})

setup_project_finalize()
