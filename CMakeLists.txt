cmake_minimum_required(VERSION 3.12)

set(TARGET_NAME display)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(${TARGET_NAME} C CXX ASM)

# Reserve 1 sector (4KB) of flash at the end for non-volatile storage
math(EXPR PICO_FLASH_SIZE_BYTES "${PICO_FLASH_SIZE_BYTES} - 4096")

# Trim data from binary that isn't going to be used. 
#This fixes a flash region overflow caused by font data from u8g2
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections -Wl,--gc-sections")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_BOARD pico_w)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
	message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()


# include(CheckIncludeFile)
# include(CheckIncludeFileCXX)
# include(CheckIncludeFiles)
# include(ExternalProject)

add_compile_options(-Wall
		-Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
		-Wno-unused-function # we have some for the docs that aren't called
		-Wno-maybe-uninitialized
		-fpermissive
		)
		
set(CMAKE_SYSTEM_NAME Generic)		

# Build info
string(TIMESTAMP BUILD_YEAR "%Y")
string(TIMESTAMP BUILD_MONTH "%m")
string(TIMESTAMP BUILD_DAY "%d")

# Force base-10 evaluation to strip leading zeros, otherwise the numbers are interpreted as octal
math(EXPR BUILD_YEAR "${BUILD_YEAR}" OUTPUT_FORMAT DECIMAL)
math(EXPR BUILD_MONTH "${BUILD_MONTH}" OUTPUT_FORMAT DECIMAL)
math(EXPR BUILD_DAY "${BUILD_DAY}" OUTPUT_FORMAT DECIMAL)

execute_process(
	COMMAND git rev-parse --short HEAD
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	OUTPUT_VARIABLE GIT_HASH
	OUTPUT_STRIP_TRAILING_WHITESPACE
)

configure_file(
	version.h.in
	version.h
	@ONLY
)


# include_directories("./bitmap")
# include_directories("./bldc")
# include_directories("./bldc/comm/")
# include_directories("./bldc/util/")
include_directories("./bldc/tests/utils_math")	# ch.h needed
include_directories("./bldc/ChibiOS_3.0.5/os/ext/CMSIS/ST/") #stm32f4xx.h needed
include_directories("./bldc/ChibiOS_3.0.5/os/ext/CMSIS/include/") #core_cm4.h needed
add_subdirectory(FatFS-SD/FatFs_SPI build)

add_executable(${TARGET_NAME}
	display.cpp
	nv_flash.cpp
	log.cpp
	crc.c 
	buffer.c
	packet.c	
	hw_config.c
	u8x8_interface.cpp
	hw_def.c
	page_ctrl.cpp
	user_cfg.c
	crash.c
	)

# pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/Button-debouncer/button_debounce.pio)
# target_sources(${TARGET_NAME} PRIVATE ./Button-debouncer/button_debounce.cpp)


### u8g2 related ###

pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/spi_tx_9bit.pio)

# Convenience to locate the library
set(U8_PATH ${CMAKE_CURRENT_LIST_DIR}/u8g2)


#add_subdirectory(${U8_PATH} u8g2)

file(GLOB U8G2_SRC u8g2/csrc/*.c)
add_library(u8g2 ${U8G2_SRC})

target_include_directories(${TARGET_NAME} PRIVATE
	u8g2/csrc
)

add_library(can2040 can2040/src/can2040.c)


set_source_files_properties(crc.c PROPERTIES LANGUAGE CXX)
set_source_files_properties(buffer.c PROPERTIES LANGUAGE CXX)
set_source_files_properties(packet.c PROPERTIES LANGUAGE CXX)

set_target_properties(${TARGET_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_options(${TARGET_NAME} PRIVATE -Xlinker --print-memory-usage)


target_compile_definitions(${TARGET_NAME} PRIVATE
    CYW43_SPI_PIO_PREFERRED_PIO=0	# pio 1 will be used for CAN, everything else goes on pio 0
)

# Needed for can2040 to find cmsis headers
target_link_libraries(can2040 cmsis_core)

# pull in common dependencies
target_link_libraries(${TARGET_NAME} 
	pico_stdlib 
	pico_multicore 
	pico_sync
	pico_malloc
	pico_cyw43_arch_none
	pico_rand
	pico_util
	hardware_i2c 
	hardware_adc 
	hardware_pwm 
	hardware_uart 
	hardware_pio
	hardware_flash
	hardware_spi
	hardware_rtc
	hardware_irq
	hardware_watchdog
	FatFs_SPI
	u8g2
	can2040
	cmsis_core
	)


# enable usb output, disable uart output
pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${TARGET_NAME})

