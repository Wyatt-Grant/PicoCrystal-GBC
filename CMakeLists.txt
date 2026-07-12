cmake_minimum_required(VERSION 3.12)

set(PROJECT_NAME PicoCrystal)

# pico-sdk already ships a board definition for this exact hardware
# (boards/pimoroni_picosystem.h) -- correct PICO_FLASH_SIZE_BYTES (16MB) and
# the right second-stage bootloader for its flash chip (W25Q080). The
# original boilerplate never set this (defaulted to the generic "pico"
# board's assumptions), which didn't matter until now: Milestone 7 does real
# flash erase/program for save persistence, where the correct boot2/flash
# timing config actually matters.
set(PICO_BOARD pimoroni_picosystem)

# --- Pico SDK (external sibling checkout, same convention as before) ---

if(NOT PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    message(WARNING "PICO_SDK_PATH not specified. Forcing fetch from git!")
    set(PICO_SDK_FETCH_FROM_GIT true)
endif()

include(pico_sdk_import.cmake)

project(${PROJECT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# --- PicoSystem SDK: NOT the external sibling checkout. This project vendors
# a narrowed, in-tree fork (picosystem/) because it needs its own main() and
# owns SCREEN itself -- see picosystem/picosystem.cmake for why. ---

include(${CMAKE_CURRENT_LIST_DIR}/picosystem/picosystem.cmake)

# --- ROM catalog: generated from whatever .gb/.gbc files sit in assets/. ---
# tools/gen_rom_data.py validates each ROM and emits rom_data.S (flash
# .incbin) plus rom_data.hpp/.cpp (the boot-menu catalog). CONFIGURE_DEPENDS
# re-runs this automatically on the next build when ROMs are added or
# removed. assets/roms.json (optional, see README) pins menu order, display
# names, and save slots.

file(GLOB ROM_FILES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/assets/*.gb
    ${CMAKE_CURRENT_SOURCE_DIR}/assets/*.gbc)
if(NOT ROM_FILES)
    message(FATAL_ERROR
        "No Game Boy ROMs found. Drop your .gb/.gbc files into assets/ and "
        "build again -- every ROM there becomes a boot-menu entry. (This "
        "repo ships no ROMs; use your own dumps or homebrew.)")
endif()
file(GLOB ROM_MANIFEST CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/assets/roms.json)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(ROM_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(ROM_GEN_SOURCES ${ROM_GEN_DIR}/rom_data.S ${ROM_GEN_DIR}/rom_data.cpp)
add_custom_command(
    OUTPUT ${ROM_GEN_SOURCES} ${ROM_GEN_DIR}/rom_data.hpp
    COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_rom_data.py
            --assets ${CMAKE_CURRENT_SOURCE_DIR}/assets --out ${ROM_GEN_DIR}
    DEPENDS ${ROM_FILES} ${ROM_MANIFEST}
            ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_rom_data.py
    COMMENT "Generating ROM catalog from assets/"
    VERBATIM)

add_executable(${PROJECT_NAME}
    main.cpp
    audio_output.cpp
    save_storage.cpp
    core/minigb_apu/minigb_apu.c
    ${ROM_GEN_SOURCES}
)

# main.cpp includes the generated rom_data.hpp.
target_include_directories(${PROJECT_NAME} PRIVATE ${ROM_GEN_DIR})

target_link_libraries(${PROJECT_NAME} PRIVATE picosystem)

# minigb_apu.h needs this consistently across every TU that includes it
# (main.cpp, audio_output.cpp/hpp, minigb_apu.c itself).
target_compile_definitions(${PROJECT_NAME} PRIVATE MINIGB_APU_AUDIO_FORMAT_S16SYS)

pico_set_linker_script(${PROJECT_NAME} ${PICOSYSTEM_LINKER_SCRIPT})

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME})
