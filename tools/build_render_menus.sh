#!/bin/sh
# Builds and runs the host-side UI renderer (see render_menus.cpp): extracts
# the @ui-draw / @menu-draw fenced drawing code out of main.cpp, compiles it
# against the harness stubs, and renders the screens as PPMs into <out_dir>
# (default: cwd).
set -e
root="$(dirname "$0")/.."
out="${1:-.}"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

sed -n '/@ui-draw-begin/,/@ui-draw-end/p' "$root/main.cpp" > "$work/ui_draw.inc"
sed -n '/@menu-draw-begin/,/@menu-draw-end/p' "$root/main.cpp" > "$work/menu_draw.inc"
g++ -std=c++17 -Wall -I"$work" "$root/tools/render_menus.cpp" -o "$work/render_menus"
mkdir -p "$out"
"$work/render_menus" "$out"
