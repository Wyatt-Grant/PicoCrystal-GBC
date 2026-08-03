# PicoCrystal fork: narrowed on purpose. We do NOT use the stock
# init()/update()/draw() model or the high-level drawing API (pen/blit/text/
# blend) -- main.cpp drives its own loop and writes SCREEN->data directly. A
# CPU/PPU/APU emulator needs lower-level control of frame timing and the
# display flip than the stock 40fps callback model can give it; see the header
# comment in main.cpp and the README.
# picosystem.cpp is deliberately excluded: it defines int main(), which
# conflicts with our own, and it statically allocates SCREEN, which main.cpp
# provides instead. blend.cpp/audio.cpp/state.cpp/primitives.cpp/text.cpp/
# assets.cpp are kept in this directory for reference but not compiled --
# they're unused (and would need drawing-state globals we don't define) and
# a linker `--gc-sections` pass would want to discard their object code
# anyway, so it's simpler to just not compile them.
add_library(picosystem INTERFACE)

pico_generate_pio_header(picosystem ${CMAKE_CURRENT_LIST_DIR}/screen.pio)
pico_generate_pio_header(picosystem ${CMAKE_CURRENT_LIST_DIR}/screen_double.pio)

target_sources(picosystem INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/hardware.cpp
  ${CMAKE_CURRENT_LIST_DIR}/utility.cpp
)

set(PICOSYSTEM_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/memmap_picosystem.ld)

target_include_directories(picosystem INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(picosystem INTERFACE pico_stdlib pico_multicore hardware_pio hardware_spi hardware_pwm hardware_dma hardware_irq hardware_adc hardware_interp hardware_clocks hardware_flash hardware_vreg)

function(pixel_double NAME)
  target_compile_options(${NAME} PRIVATE -DPIXEL_DOUBLE)
endfunction()

function(disable_startup_logo NAME)
  target_compile_options(${NAME} PRIVATE -DNO_STARTUP_LOGO)
endfunction()

function(no_font NAME)
  target_compile_options(${NAME} PRIVATE -DNO_FONT)
endfunction()

function(no_spritesheet NAME)
  target_compile_options(${NAME} PRIVATE -DNO_SPRITESHEET)
endfunction()

function(no_overclock NAME)
  target_compile_options(${NAME} PRIVATE -DNO_OVERCLOCK)
endfunction()