cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(osciloscopio_usb_otg C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(osciloscopio
    main_v2_2_usb_otg.c
)

target_link_libraries(osciloscopio
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_i2c
    hardware_uart
    hardware_sync
    hardware_flash
    pico_multicore
)

pico_enable_stdio_usb(osciloscopio 1)
pico_enable_stdio_uart(osciloscopio 0)

pico_add_extra_outputs(osciloscopio)
