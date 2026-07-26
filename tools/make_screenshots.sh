#!/bin/sh
# Regenerates the README gallery in screenshots/.
#
#   tools/make_screenshots.sh [rom [frames [script]]]
#
# Two host-side tools do the work, neither of which needs hardware:
#
#   grab_frame.c      boots <rom> in Walnut-CGB for <frames> frames (driving
#                     the joypad with <script>, see the tool's header) and
#                     dumps the 160x144 GBC frame it lands on;
#   render_menus.cpp  compiles the *actual* UI drawing code out of main.cpp
#                     and renders every menu screen -- plus that game frame
#                     under the device's 1.5x scaler and status bar.
#
# The PPMs they emit are then upscaled 2x (nearest-neighbour, so the pixels
# stay square) into screenshots/*.png.
#
# Defaults reproduce the committed set: the Polished Crystal title screen,
# which is where pokemon_crystal.gbc lands ~3300 frames in with no input.
set -e

root="$(dirname "$0")/.."
rom="${1:-$root/assets/pokemon_crystal.gbc}"
frames="${2:-3300}"
script="${3:-}"
out="$root/screenshots"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

if [ ! -f "$rom" ]; then
	echo "no ROM at $rom -- pass one: tools/make_screenshots.sh <rom> [frames] [script]" >&2
	exit 1
fi

echo "== grabbing game frame ($frames frames of $(basename "$rom"))"
cc -O2 -o "$work/grab_frame" "$root/tools/grab_frame.c" -lm
"$work/grab_frame" "$rom" "$work/game.ppm" "$frames" "$script"

echo "== rendering UI screens"
"$root/tools/build_render_menus.sh" "$work/ppm" "$work/game.ppm" >/dev/null

echo "== writing $out"
mkdir -p "$out"
python3 "$root/tools/upscale_png.py" 2 "$work/png" "$work"/ppm/*.ppm >/dev/null

# The renderer's names are keyed to the code paths they exercise; the gallery's
# are keyed to what a reader sees. Only the screens listed here are published --
# the rest of the renderer's output stays a dev-time eyeball aid.
publish() {
	cp "$work/png/$1.png" "$out/$2.png"
	echo "  $2.png"
}

publish boot                boot-menu
publish settings            settings
publish clock               clock
publish in_game             in-game
publish in_game_fullscreen  in-game-fullscreen
publish boot_light_mint     boot-menu-light
publish settings_light_mint settings-light

for t in "$work"/png/theme_*.png; do
	name="$(basename "$t" .png)"
	publish "$name" "$(echo "$name" | tr '_' '-')"
done

echo "done."
