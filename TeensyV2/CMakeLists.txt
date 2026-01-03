cmake_minimum_required(VERSION 3.8)
project(teensy_v2 VERSION 2.0.0 LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Include directories for headers
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/common
  ${CMAKE_CURRENT_SOURCE_DIR}/modules
)

# Define source files for testing
# Note: We don't build the actual firmware here, just the testable components
# Phase 1: Only build what's needed for temperature_monitor test
set(TEENSY_SOURCES
  # Mock infrastructure (Arduino API for PC)
  test/mocks/Arduino.cpp
  test/mocks/serial_manager_stub.cpp  # Stub SerialManager
  
  # Core infrastructure
  common/core/module.cpp
  # common/core/serial_manager.cpp  # Replaced by stub above
  
  # Sensors
  modules/sensors/temperature_monitor.cpp
  
  # Safety - enabled for SafetyCoordinator tests
  modules/safety/safety_coordinator.cpp
  
  # Motors/RoboClaw - PHASE 2 (needs interface abstraction)
  # modules/roboclaw/roboclaw_monitor.cpp
  
  # Performance monitoring - PHASE 2 (needs RoboClaw)
  # modules/performance/performance_monitor.cpp
)

# Create a library from the Teensy sources for testing
# This library is NOT for deployment, only for PC-based unit tests
add_library(${PROJECT_NAME}_core STATIC ${TEENSY_SOURCES})
target_compile_definitions(${PROJECT_NAME}_core PUBLIC 
  UNIT_TEST
  BOARD_ID=1
  BOARD_1_HAS_SD_LOGGING=1
  BOARD_1_HAS_PERFORMANCE_MONITOR=1
  BOARD_1_HAS_SAFETY_COORDINATOR=1
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/modules>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/mocks>
  $<INSTALL_INTERFACE:include>
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Temperature monitor tests
  ament_add_gtest(test_temperature_monitor
    test/temperature_monitor_test.cpp
  )
  
  if(TARGET test_temperature_monitor)
    target_link_libraries(test_temperature_monitor
      ${PROJECT_NAME}_core
    )
    target_include_directories(test_temperature_monitor PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/test/mocks
    )
  endif()
  
  # Safety coordinator tests
  ament_add_gtest(test_safety_coordinator
    test/safety_coordinator_test.cpp
  )
  
  if(TARGET test_safety_coordinator)
    target_link_libraries(test_safety_coordinator
      ${PROJECT_NAME}_core
    )
    target_include_directories(test_safety_coordinator PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/test/mocks
    )
  endif()
  
endif()

ament_package()
