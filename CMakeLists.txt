cmake_minimum_required(VERSION 3.13)
if (DEFINED MCU)
	# Setting up the toolchain
	set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
endif()
# Project information
project(BME280
	VERSION 1.0
	DESCRIPTION "BME-280 sensor driver"
	LANGUAGES C CXX
)
if (DEFINED MCU)
	# Grab the processor and set up definitions and compile options
	include(${CMAKE_SOURCE_DIR}/cmake/config_mcu.cmake)
	configMcu(${MCU})
	# Fetch types
	include(FetchContent)
	FetchContent_Declare(
		types
		GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/types.git
	)
	FetchContent_MakeAvailable(types)
	# Fetch core
	FetchContent_Declare(
		core
		GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/core.git
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
			types
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
		# Add the spi example target
		add_executable(bme280_example examples/bme280_example.cc)
		# Add the includes
		target_include_directories(bme280_example PUBLIC 
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<INSTALL_INTERFACE:include>
		)
		# Link libraries to the example target
		target_link_libraries(bme280_example
			PRIVATE 
			bme280
		)
		# Add hex and upload targets
		include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
		FlashMcu(bme280_example ${MCU})
	endif()
endif()
