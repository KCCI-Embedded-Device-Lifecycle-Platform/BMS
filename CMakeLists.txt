cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Embedded_Device_Lifecycle_BMS)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
#   BMS/ 는 계층 구조(common -> hw -> dev -> app)를 그대로 유지한다.
#   새 모듈을 추가하면 해당 계층 블록에 한 줄만 넣으면 된다.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # hw : HAL 래퍼 (여기만 stm32f4xx_hal.h 를 안다)
    BMS/hw/dbg.c
    BMS/hw/hw_adc.c
    BMS/hw/hw_gpio.c
    BMS/hw/hw_i2c.c
    BMS/hw/hw_tick.c
    BMS/hw/hw_uart.c

    # dev : 부품 단위 드라이버
    BMS/dev/acs712.c
    BMS/dev/cell_adc.c
    # 팩 전류 센서. 2026-08-14 INA226 입고로 대체 경로(ina219.c)는 삭제했다.
    #   -> app 계층은 여전히 dev/pack_sensor.h 만 보므로, 칩을 갈면 그 헤더와 이 한 줄이 전부다.
    BMS/dev/ina226.c
    BMS/dev/ntc.c
    BMS/dev/oled.c
    BMS/dev/status_led.c

    # app : BMS 도메인 로직
    BMS/app/bms_app.c
    BMS/app/bms_can.c
    BMS/app/bms_fault.c
    BMS/app/bms_link.c
    BMS/app/bms_soc.c
    BMS/app/bms_state.c
    BMS/app/bms_ui.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    BMS/common
    BMS/hw
    BMS/dev
    BMS/app
)

# F446 로 옮기면서 SPI2 / I2C1 은 .ioc 에 정식으로 들어갔다.
#   -> stm32f4xx_hal_spi.c 는 cmake/stm32cubemx/CMakeLists.txt 의 STM32_Drivers
#      오브젝트 라이브러리가 이미 컴파일한다. 여기서 또 넣으면 같은 심볼이
#      실행 파일에 두 번 들어가 "multiple definition of HAL_SPI_Init" 로 링크가 깨진다.
#   -> HAL_SPI_MODULE_ENABLED 도 stm32f4xx_hal_conf.h 가 이미 정의한다.
# 그래서 F411 시절의 수동 추가 블록은 제거했다.

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
