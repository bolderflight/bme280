# v3.14 required for FetchContent_MakeAvailable
cmake_minimum_required(VERSION 3.14)
if (DEFINED MCU)
  include(FetchContent)
  FetchContent_Declare(
    mcu_support
    GIT_REPOSITORY https://github.com/bolderflight/mcu-support.git
    GIT_TAG v1.1.0
  )
  FetchContent_MakeAvailable(mcu_support)
  # Setting up the toolchain
  set(CMAKE_TOOLCHAIN_FILE "${mcu_support_SOURCE_DIR}/cmake/cortex.cmake")
  # Project information
  project(Bme280
    VERSION 5.2.6
    DESCRIPTION "BME-280 sensor driver"
    LANGUAGES C CXX
  )
  # Grab the processor and set up definitions and compile options
  include(${mcu_support_SOURCE_DIR}/cmake/config_mcu.cmake)
  configMcu(${MCU} ${mcu_support_SOURCE_DIR})
  # Fetch core
  FetchContent_Declare(
    core
    GIT_REPOSITORY https://github.com/bolderflight/core.git
    GIT_TAG v3.1.3
  )
  FetchContent_MakeAvailable(core)
  # Add the library target
  add_library(bme280
    src/bme280.cpp
    src/bme280.h
    src/bst/bme280_defs.h
    src/bst/bme280.h
    src/bst/bme280.c
  )
  # Link libraries
  target_link_libraries(bme280
    PUBLIC
      core
  )
  # Setup include directories 
  target_include_directories(bme280 PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )
  # Example and test if this project is built separately
  if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
    # Add the i2c example target
    add_executable(i2c_example examples/cmake/i2c.cc)
    # Add the includes
    target_include_directories(i2c_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(i2c_example
      PRIVATE
        bme280
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(i2c_example ${MCU} ${mcu_support_SOURCE_DIR})

    # Add the spi example target
    add_executable(spi_example examples/cmake/spi.cc)
    # Add the includes
    target_include_directories(spi_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(spi_example
      PRIVATE
        bme280
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(spi_example ${MCU} ${mcu_support_SOURCE_DIR})
  endif()
endif()
