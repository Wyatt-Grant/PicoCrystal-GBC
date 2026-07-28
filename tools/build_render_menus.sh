#!/bin/sh
# Builds and runs the host-side UI renderer (see render_menus.cpp): extracts
# the @ui-draw / @menu-draw fenced drawing code out of main.cpp, compiles it
# against the harness stubs, and renders the screens as PPMs into <out_dir>
# (default: cwd).
#
#   build_render_menus.sh [out_dir [game.ppm]]
#
# An optional 160x144 GBC frame (tools/grab_frame.c) adds the in-game screens,
# with the frame run through the device's 1.5x scaler under the status bar.
#
# If the firmware has been built (build/generated/rom_data.cpp exists, or
# $ROM_DATA_CPP points at one), the boot menu renders that real catalog --
# your ROM names, cart tints and assets/icons.png art -- instead of the
# placeholder list in render_menus.cpp. The ROM bytes are not needed: the
# catalog is rewritten below to drop the .incbin symbols the menus never read.
set -e
root="$(dirname "$0")/.."
out="${1:-.}"
game="${2:-}"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

sed -n '/@ui-draw-begin/,/@ui-draw-end/p' "$root/main.cpp" > "$work/ui_draw.inc"
sed -n '/@menu-draw-begin/,/@menu-draw-end/p' "$root/main.cpp" > "$work/menu_draw.inc"

real_catalog="${ROM_DATA_CPP:-$root/build/generated/rom_data.cpp}"
catalog_def=""
if [ -f "$real_catalog" ]; then
	# rom_data.cpp minus its ROM payloads: the size expression and the
	# `data` pointer both name .incbin symbols that only exist in the
	# firmware link, and no menu draws either field.
	sed -e '/#include "rom_data.hpp"/d' \
	    -e 's/(uint32_t)(rom_\([0-9]*\)_data_end - rom_\1_data)/0/g' \
	    -e 's/rom_\([0-9]*\)_data\([^_a-zA-Z0-9]\)/nullptr\2/g' \
	    -e 's/^const rom_entry_t rom_catalog\[ROM_COUNT\] = {/static const rom_entry_t rom_catalog[] = {/' \
	    "$real_catalog" > "$work/rom_catalog.inc"
	catalog_def="-DREAL_ROM_CATALOG"
	echo "using real ROM catalog from $real_catalog" >&2
fi

g++ -std=c++17 -Wall $catalog_def -I"$work" "$root/tools/render_menus.cpp" -o "$work/render_menus"
mkdir -p "$out"
"$work/render_menus" "$out" ${game:+"$game"}
