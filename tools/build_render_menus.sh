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
set -e
root="$(dirname "$0")/.."
out="${1:-.}"
game="${2:-}"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

sed -n '/@ui-draw-begin/,/@ui-draw-end/p' "$root/main.cpp" > "$work/ui_draw.inc"
sed -n '/@menu-draw-begin/,/@menu-draw-end/p' "$root/main.cpp" > "$work/menu_draw.inc"
g++ -std=c++17 -Wall -I"$work" "$root/tools/render_menus.cpp" -o "$work/render_menus"
mkdir -p "$out"
"$work/render_menus" "$out" ${game:+"$game"}
