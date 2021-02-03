cmake_minimum_required(VERSION 3.13)
if (DEFINED MCU)
	# Setting up the toolchain
	set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
endif()
# Project information
project(Bme280
	VERSION 2.0.0
	DESCRIPTION "BME-280 sensor driver"
	LANGUAGES CXX
)
if (DEFINED MCU)
	# Grab the processor and set up definitions and compile options
	include(${CMAKE_SOURCE_DIR}/cmake/config_mcu.cmake)
	configMcu(${MCU})
	include(FetchContent)
	# Fetch core
	FetchContent_Declare(
		core
		GIT_REPOSITORY 	https://gitlab.com/bolderflight/software/core.git
		GIT_TAG v2.0.3
	)
	FetchContent_MakeAvailable(core)
	# Add the library target
	add_library(bme280
		src/bme280/bme280.cc
		include/bme280/bme280.h
	)
	# Link libraries
	target_link_libraries(bme280
		PUBLIC
			core
	)
	# Setup include directories 
	target_include_directories(bme280 PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	)

endif()
# Example and test if this project is built separately
if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
	if (DEFINED MCU)
		# Add the i2c example target
		add_executable(i2c_example examples/i2c.cc)
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
		include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
		FlashMcu(i2c_example ${MCU})
		# Add the spi example target
		add_executable(spi_example examples/spi.cc)
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
		include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
		FlashMcu(spi_example ${MCU})
	endif()
endif()
