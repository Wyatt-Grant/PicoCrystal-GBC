#!/usr/bin/env python3
"""Generate the embedded ROM catalog (rom_data.S/.hpp/.cpp) from assets/.

Every *.gb / *.gbc file in the assets directory becomes a boot-menu entry:
the .S embeds the ROM bytes into flash via .incbin, the .hpp/.cpp expose the
rom_catalog[] that main.cpp's boot menu and save_storage index. Run by CMake
(see CMakeLists.txt) -- users just drop ROM files in assets/ and build.

Each ROM's Game Boy header is validated up front (checksum, mapper support,
cart-RAM size) so a bad file fails the build with a readable message instead
of hanging the handheld at boot.

Menu order, display names, and save slots default to alphabetical filename
order. An optional assets/roms.json pins any of them (see the README) --
pinning save slots is what keeps existing on-device saves attached
to their game when the ROM list changes, since a game's save lives in the
flash region picked by its slot number, not its menu position.

An optional assets/icons.png (8px wide, one 8x8 tile per save slot stacked
top to bottom) replaces the generic cartridge glyph in the boot menu with
per-game art. Tiles are keyed by save slot, not menu position, so an icon
stays with its game for the same reason its save does.
"""

import argparse
import json
import os
import re
import struct
import sys
import zlib

# Keep in sync with save_storage.hpp's MAX_ROM_SAVE_SLOTS: save regions are
# stacked down from the top of the 16MB flash, one 72KB region per slot plus
# one settings sector, and 14 is the most that fits in the 1MB above the
# linker script's 15MB FLASH (code + ROMs) region.
MAX_ROM_SAVE_SLOTS = 14

FLASH_REGION_BYTES = 15 * 1024 * 1024  # memmap_picosystem.ld FLASH LENGTH
# Firmware (emulator + picosystem + save/audio code) is well under this; the
# check exists to turn "linker: region FLASH overflowed" into a message that
# says which ROM to drop.
FIRMWARE_MARGIN_BYTES = 768 * 1024

# Cart-RAM ceiling: main.cpp's static cart_ram buffer and save_storage's
# per-slot payload are both sized for 32KB (4 banks x 8KB).
MAX_CART_RAM_BYTES = 32 * 1024

# Boot-menu icon sheet: assets/icons.png is ICON_PX wide and holds one
# ICON_PX-square tile per save slot, stacked top to bottom (so the full sheet
# is 8x112). main.cpp's draw_rom_icon() draws a tile at 2x.
ICON_SHEET = "icons.png"
ICON_PX = 8

# Mirrors walnut_cgb.h's gb_init() tables: cartridge-type byte (0x147) ->
# MBC number, -1 if the core rejects it (GB_INIT_CARTRIDGE_UNSUPPORTED).
CART_MBC = [
    0, 1, 1, 1, -1, 2, 2, -1, 0, 0, -1, 0, 0, 0, -1, 3,
    3, 3, 3, 3, -1, -1, -1, -1, -1, 5, 5, 5, 5, 5, 5, -1,
]
CART_HAS_RAM = [
    0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
]
# RAM-size byte (0x149) -> number of 8KB banks.
NUM_RAM_BANKS = [0, 1, 1, 4, 16, 8]
# ROM-size byte (0x148) indexes a 9-entry bank table in the core.
MAX_ROM_SIZE_CODE = 8


def fail(msg):
    sys.stderr.write("gen_rom_data: error: %s\n" % msg)
    sys.exit(1)


def display_name(stem):
    """Filename stem -> boot-menu name (status_glyph charset: A-Z 0-9 space)."""
    name = re.sub(r"[^A-Z0-9]+", " ", stem.upper()).strip()
    if not name:
        name = "UNTITLED"
    return name[:24].strip()


def parse_color(value, where):
    """roms.json "color" -> packed RGBA4444 int (or None if unset).

    Accepts a hex RGB string: "#RRGGBB" or the shorthand "#RGB", with or
    without the leading '#'. The 8-bit channels are truncated to the screen's
    4 bits each and packed to match main.cpp's status_rgb() layout (R bits
    0-3, A 4-7 forced opaque, B 8-11, G 12-15) so the generated color_t can be
    used by the boot menu directly.
    """
    if value is None:
        return None
    if not isinstance(value, str):
        fail("%s: \"color\" must be a hex RGB string like \"#3fa9f5\"" % where)
    s = value.strip().lstrip("#")
    if len(s) == 3:
        s = "".join(c * 2 for c in s)
    if len(s) != 6 or re.search(r"[^0-9a-fA-F]", s):
        fail("%s: color '%s' must be a hex RGB string like \"#3fa9f5\" or "
             "\"#39f\"" % (where, value))
    r4 = int(s[0:2], 16) >> 4
    g4 = int(s[2:4], 16) >> 4
    b4 = int(s[4:6], 16) >> 4
    return r4 | 0xF0 | (b4 << 8) | (g4 << 12)


def pack_rgba4444(r, g, b, a):
    """8-bit RGBA -> main.cpp's color_t bit layout (R 0-3, A 4-7, B 8-11, G 12-15).

    Alpha is 1-bit: draw_rom_icon() writes tile pixels straight into the
    framebuffer with no blending, so anything under half opaque is dropped
    (packs to 0, which the drawing loop skips) and everything else is solid.
    Colors round to the nearest 4-bit level rather than truncating, so a 0xEE
    channel lands on 15/15 instead of 14/15.
    """
    if a < 128:
        return 0
    def q(v):
        return (v * 15 + 127) // 255
    return q(r) | 0xF0 | (q(b) << 8) | (q(g) << 12)


def read_png_rgba(path):
    """Minimal stdlib PNG reader -> (width, height, [(r,g,b,a), ...]).

    Only what an icon sheet needs: 8 bits per channel, non-interlaced, any of
    the five color types. Exports outside that are rejected with a message
    naming the fix rather than a stack trace.
    """
    with open(path, "rb") as f:
        data = f.read()
    base = os.path.basename(path)
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        fail("%s is not a PNG file" % base)

    ihdr = plte = trns = None
    idat = []
    pos = 8
    while pos + 8 <= len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        ctype = data[pos + 4:pos + 8]
        chunk = data[pos + 8:pos + 8 + length]
        pos += 12 + length  # 4 length + 4 type + data + 4 CRC
        if ctype == b"IHDR":
            ihdr = struct.unpack(">IIBBBBB", chunk)
        elif ctype == b"PLTE":
            plte = chunk
        elif ctype == b"tRNS":
            trns = chunk
        elif ctype == b"IDAT":
            idat.append(chunk)
        elif ctype == b"IEND":
            break
    if ihdr is None or not idat:
        fail("%s is a truncated or corrupt PNG (no image data)" % base)

    w, h, depth, color_type, _comp, _filt, interlace = ihdr
    if depth != 8 or interlace or color_type not in (0, 2, 3, 4, 6):
        fail("%s must be a non-interlaced, 8-bit-per-channel PNG (this one is "
             "%d-bit, color type %d%s) -- re-export it as 8-bit RGBA"
             % (base, depth, color_type, ", interlaced" if interlace else ""))
    if color_type == 3 and plte is None:
        fail("%s is a paletted PNG with no palette chunk" % base)

    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}[color_type]
    stride = w * channels
    raw = zlib.decompress(b"".join(idat))
    if len(raw) < h * (stride + 1):
        fail("%s is truncated (%d image bytes, expected %d)"
             % (base, len(raw), h * (stride + 1)))

    # Undo the per-scanline filters (PNG spec 9.2); `prev` is the already
    # reconstructed line above, `bpp` the byte distance to the pixel at left.
    rows = []
    prev = bytearray(stride)
    p = 0
    for _ in range(h):
        ftype = raw[p]
        p += 1
        line = bytearray(raw[p:p + stride])
        p += stride
        for i in range(stride):
            left = line[i - channels] if i >= channels else 0
            up = prev[i]
            upleft = prev[i - channels] if i >= channels else 0
            if ftype == 1:
                line[i] = (line[i] + left) & 0xFF
            elif ftype == 2:
                line[i] = (line[i] + up) & 0xFF
            elif ftype == 3:
                line[i] = (line[i] + ((left + up) >> 1)) & 0xFF
            elif ftype == 4:  # Paeth
                pa, pb, pc = abs(up - upleft), abs(left - upleft), \
                    abs(left + up - 2 * upleft)
                pred = left if (pa <= pb and pa <= pc) else (up if pb <= pc else upleft)
                line[i] = (line[i] + pred) & 0xFF
            elif ftype != 0:
                fail("%s uses unknown PNG filter type %d" % (base, ftype))
        rows.append(line)
        prev = line

    pixels = []
    for line in rows:
        for x in range(w):
            s = line[x * channels:(x + 1) * channels]
            if color_type == 0:
                pixels.append((s[0], s[0], s[0], 255))
            elif color_type == 2:
                pixels.append((s[0], s[1], s[2], 255))
            elif color_type == 3:
                i = s[0]
                if (i + 1) * 3 > len(plte):
                    fail("%s references palette entry %d, past the end of its "
                         "palette" % (base, i))
                a = trns[i] if trns and i < len(trns) else 255
                pixels.append((plte[i * 3], plte[i * 3 + 1], plte[i * 3 + 2], a))
            elif color_type == 4:
                pixels.append((s[0], s[0], s[0], s[1]))
            else:
                pixels.append((s[0], s[1], s[2], s[3]))
    return w, h, pixels


def read_icons(assets_dir):
    """assets/icons.png -> {save_slot: [ICON_PX*ICON_PX packed pixels]}.

    Missing sheet, or a sheet with fewer tiles than there are slots, is fine:
    slots without art fall back to main.cpp's generic cartridge glyph. So does
    a fully transparent tile, which is how you leave a gap for a slot you
    haven't drawn yet.
    """
    path = os.path.join(assets_dir, ICON_SHEET)
    if not os.path.exists(path):
        return {}

    w, h, pixels = read_png_rgba(path)
    if w != ICON_PX or h % ICON_PX or h == 0:
        fail("%s must be %dpx wide and a whole number of %dpx tiles tall "
             "(one per save slot, so %dx%d for all %d) -- this one is %dx%d"
             % (ICON_SHEET, ICON_PX, ICON_PX, ICON_PX,
                ICON_PX * MAX_ROM_SAVE_SLOTS, MAX_ROM_SAVE_SLOTS, w, h))
    tiles = h // ICON_PX
    if tiles > MAX_ROM_SAVE_SLOTS:
        fail("%s holds %d tiles but there are only %d save slots -- trim it to "
             "%dx%d" % (ICON_SHEET, tiles, MAX_ROM_SAVE_SLOTS, ICON_PX,
                        ICON_PX * MAX_ROM_SAVE_SLOTS))

    icons = {}
    for slot in range(tiles):
        tile = [pack_rgba4444(*pixels[(slot * ICON_PX + y) * w + x])
                for y in range(ICON_PX) for x in range(ICON_PX)]
        if any(tile):  # all-transparent tile == "no icon for this slot"
            icons[slot] = tile
    return icons


def validate_rom(path):
    """Return (size, warnings) or die with a message naming the file."""
    size = os.path.getsize(path)
    base = os.path.basename(path)
    if size < 0x150:
        fail("%s is too small (%d bytes) to be a Game Boy ROM" % (base, size))
    with open(path, "rb") as f:
        hdr = f.read(0x150)

    checksum = 0
    for i in range(0x134, 0x14D):
        checksum = (checksum - hdr[i] - 1) & 0xFF
    if checksum != hdr[0x14D]:
        fail("%s failed the Game Boy header checksum -- not a valid "
             ".gb/.gbc ROM (truncated download or wrong file type?)" % base)

    cart_type = hdr[0x147]
    if cart_type >= len(CART_MBC) or CART_MBC[cart_type] == -1:
        fail("%s uses cartridge type 0x%02X, which the Walnut-CGB core "
             "does not support (supported: ROM-only, MBC1/2/3/5)"
             % (base, cart_type))

    ram_code = hdr[0x149]
    if ram_code >= len(NUM_RAM_BANKS):
        fail("%s declares an invalid cart-RAM size code 0x%02X" % (base, ram_code))
    if CART_HAS_RAM[cart_type] and NUM_RAM_BANKS[ram_code] * 8 * 1024 > MAX_CART_RAM_BYTES:
        fail("%s declares %dKB of cart RAM; this build supports at most "
             "%dKB (main.cpp's cart_ram buffer and the per-game flash save "
             "regions are sized for 32KB)"
             % (base, NUM_RAM_BANKS[ram_code] * 8, MAX_CART_RAM_BYTES // 1024))

    warnings = []
    if hdr[0x148] > MAX_ROM_SIZE_CODE:
        fail("%s declares an invalid ROM size code 0x%02X" % (base, hdr[0x148]))
    declared = (32 * 1024) << hdr[0x148]
    if declared != size:
        warnings.append("%s: file is %d bytes but its header declares %d "
                        "(overdump/padding? it will still boot)" % (base, size, declared))
    return size, warnings


def parse_cfg(cfg_path, roms_by_file, warnings):
    """assets/roms.json: {"roms": [{"file", "name"?, "slot"?}, ...]}.

    A bare JSON array of those entry objects is also accepted. `name`,
    `slot`, and `color` are optional (omit or set to null). Returns the
    ordered entry list [(filename, name_or_None, slot_or_None,
    color_or_None)]. Entries whose file is
    absent from assets/ are skipped with a warning, not an error -- they are
    dormant pins: removing a game shouldn't force a manifest edit, and
    re-adding the file later reactivates its pinned name/slot (reattaching
    its old save).
    """
    with open(cfg_path) as f:
        try:
            doc = json.load(f)
        except ValueError as e:
            fail("roms.json is not valid JSON: %s" % e)

    rom_list = doc.get("roms", []) if isinstance(doc, dict) else doc
    if not isinstance(rom_list, list):
        fail("roms.json must be an object with a \"roms\" array (or a bare "
             "array of entries)")

    entries = []
    dormant = {}  # slot -> filename, pins whose file is absent
    seen = set()
    for idx, item in enumerate(rom_list):
        where = "roms.json entry %d" % idx
        if not isinstance(item, dict):
            fail("%s must be an object like {\"file\": \"game.gbc\", "
                 "\"name\": \"NAME\", \"slot\": 0}" % where)
        fname = item.get("file")
        if not isinstance(fname, str) or not fname:
            fail("%s is missing a string \"file\" field" % where)
        where = "roms.json entry for '%s'" % fname

        # Slot is parsed before the file-presence check so a dormant pin can
        # keep its slot reserved. bool is an int subclass in Python, so reject
        # true/false explicitly.
        slot = item.get("slot")
        if slot is not None:
            if isinstance(slot, bool) or not isinstance(slot, int):
                fail("%s: save slot must be a whole number" % where)
            if not 0 <= slot < MAX_ROM_SAVE_SLOTS:
                fail("%s: save slot %d out of range 0..%d"
                     % (where, slot, MAX_ROM_SAVE_SLOTS - 1))

        if fname not in roms_by_file:
            warnings.append(
                "roms.json: '%s' is not in assets/ -- entry skipped (fine if "
                "you removed the game; a typo here means the real file gets "
                "an auto name and slot instead of these pinned ones)" % fname)
            if slot is not None:
                dormant[slot] = fname
            continue
        if fname in seen:
            fail("roms.json: '%s' listed twice" % fname)
        seen.add(fname)

        name = item.get("name")
        if name is not None:
            if not isinstance(name, str):
                fail("%s: \"name\" must be a string" % where)
            # Lowercase is accepted for convenience and folded to upper, since
            # the boot menu font only has uppercase glyphs.
            name = name.upper()
            bad = re.sub(r"[A-Z0-9 ]", "", name)
            if bad:
                fail("%s: name '%s' contains '%s' -- the boot menu font only "
                     "has A-Z, 0-9, and space" % (where, name, bad))
            name = name[:24].strip() or None

        color = parse_color(item.get("color"), where)
        entries.append((fname, name, slot, color))
    return entries, dormant


def build_catalog(assets_dir):
    """Return ordered [(path, display_name, save_slot, size, color, icon)] plus
    warnings. `icon` is the slot's 8x8 tile from assets/icons.png, or None."""
    roms_by_file = {}
    for entry in sorted(os.listdir(assets_dir)):
        if entry.lower().endswith((".gb", ".gbc")) and \
           os.path.isfile(os.path.join(assets_dir, entry)):
            roms_by_file[entry] = os.path.join(assets_dir, entry)

    if not roms_by_file:
        fail("no .gb/.gbc files found in %s -- drop your Game Boy (Color) "
             "ROM files there and rebuild" % assets_dir)

    warnings = []
    cfg_path = os.path.join(assets_dir, "roms.json")
    cfg, dormant = (parse_cfg(cfg_path, roms_by_file, warnings)
                    if os.path.exists(cfg_path) else ([], {}))

    # cfg entries first (in cfg order), then any remaining files alphabetically.
    ordered = list(cfg)
    in_cfg = {fname for fname, _, _, _ in cfg}
    ordered += [(f, None, None, None) for f in sorted(roms_by_file) if f not in in_cfg]

    if len(ordered) > MAX_ROM_SAVE_SLOTS:
        fail("%d ROMs found, but at most %d fit (each needs a flash save "
             "region; see save_storage.hpp's MAX_ROM_SAVE_SLOTS)"
             % (len(ordered), MAX_ROM_SAVE_SLOTS))

    # Pinned slots claim theirs first; the rest fill lowest-free in order.
    # Dormant pins (cfg entries whose file is currently absent) keep their
    # slot reserved so an auto-assigned game can't take it -- otherwise
    # re-adding the old file would evict the newcomer from the slot its save
    # now lives in.
    taken = dict(dormant)
    for fname, _, slot, _ in ordered:
        if slot is not None:
            if slot in taken:
                fail("roms.json: save slot %d assigned to both '%s' and '%s'"
                     % (slot, taken[slot], fname))
            taken[slot] = fname

    icons = read_icons(assets_dir)

    catalog = []
    total = 0
    for fname, name, slot, color in ordered:
        path = roms_by_file[fname]
        size, warns = validate_rom(path)
        warnings += warns
        if slot is None:
            slot = next((s for s in range(MAX_ROM_SAVE_SLOTS) if s not in taken),
                        None)
            if slot is None:
                fail("no free save slot for '%s': all %d are pinned in "
                     "roms.json (including entries for removed games, whose "
                     "slots stay reserved) -- free one up" %
                     (fname, MAX_ROM_SAVE_SLOTS))
            taken[slot] = fname
        if name is None:
            name = display_name(os.path.splitext(fname)[0])
        if '"' in path or "\\" in path or "\n" in path:
            fail("path '%s' contains characters .incbin can't take -- "
                 "please rename" % path)
        catalog.append((path, name, slot, size, color, icons.get(slot)))
        total += size

    # One nudge for a half-drawn sheet: a blank tile is a legitimate "use the
    # cart glyph here", but it's also what a sheet drawn in menu order rather
    # than slot order looks like, and that's worth catching before flashing.
    if icons:
        bare = [n for _, n, s, _, _, _ in catalog if s not in icons]
        if bare:
            warnings.append("%s has no tile for %s -- %s the default cartridge "
                            "icon (tiles are keyed by save slot, listed below)"
                            % (ICON_SHEET, ", ".join(bare),
                               "they keep" if len(bare) > 1 else "it keeps"))

    if total + FIRMWARE_MARGIN_BYTES > FLASH_REGION_BYTES:
        listing = "\n".join("  %8d KB  %s" % (s // 1024, os.path.basename(p))
                            for p, _, _, s, _, _ in catalog)
        fail("ROMs total %.2f MB, but only ~%.2f MB of flash is available "
             "for them (16MB chip minus firmware and save storage). Remove "
             "some:\n%s"
             % (total / 1048576.0,
                (FLASH_REGION_BYTES - FIRMWARE_MARGIN_BYTES) / 1048576.0,
                listing))
    return catalog, warnings


HEADER_NOTE = """\
// Generated by tools/gen_rom_data.py from the ROM files in assets/ --
// do not edit; changes belong in the generator or assets/roms.json.
"""


def write_if_changed(path, content):
    """Skip the write when identical so ninja/make don't rebuild dependents."""
    try:
        with open(path) as f:
            if f.read() == content:
                return
    except OSError:
        pass
    with open(path, "w") as f:
        f.write(content)


def emit(catalog, out_dir):
    n = len(catalog)

    s = ["/* Generated by tools/gen_rom_data.py -- do not edit. Embeds each\n"
         " * ROM from assets/ into flash via .incbin (a multi-megabyte hex-\n"
         " * literal C array would be far slower to compile). */\n\n"
         ".section .rodata\n"]
    for i, (path, name, _, _, _, _) in enumerate(catalog):
        s.append(".align 4\n"
                 ".global rom_%d_data\n"
                 "rom_%d_data:                /* %s */\n"
                 ".incbin \"%s\"\n"
                 ".global rom_%d_data_end\n"
                 "rom_%d_data_end:\n\n" % (i, i, name, path, i, i))

    h = [HEADER_NOTE,
         "#pragma once\n#include <cstdint>\n\n"
         "// ROM bytes live in flash (.rodata, defined in generated rom_data.S)\n"
         "// and are read in place via XiP -- never copied to RAM.\n"]
    for i in range(n):
        h.append("extern \"C\" const uint8_t rom_%d_data[], rom_%d_data_end[];\n"
                 % (i, i))
    h.append("""
// One entry per bootable ROM, shown on the boot-select screen in this order.
// `save_slot` indexes save_storage's per-ROM flash reservation (see
// save_storage.hpp) -- it must stay stable per game across rebuilds or the
// game boots with a fresh save; pin slots in assets/roms.json when changing
// the ROM list on a device that already has saves.
struct rom_entry_t {
	const char *name; // status_glyph() charset only (A-Z, 0-9, space)
	const uint8_t *data;
	uint32_t size;
	uint32_t save_slot;
	// Cart-label tint (RGBA4444) from roms.json "color"; 0 = default accent.
	uint16_t label_color;
	// This slot's 8x8 RGBA4444 tile from assets/icons.png, row-major, or
	// nullptr to fall back to the generic cartridge glyph. Pixels that were
	// transparent in the PNG are 0 and get skipped when drawn.
	const uint16_t *icon;
};

constexpr uint32_t ROM_COUNT = %d;
extern const rom_entry_t rom_catalog[ROM_COUNT];
""" % n)

    c = [HEADER_NOTE, "#include \"rom_data.hpp\"\n\n"]
    # Icon tiles first: one array per game that has art in assets/icons.png,
    # named for the save slot it came from. const, so they stay in flash.
    for i, (_, name, slot, _, _, icon) in enumerate(catalog):
        if icon is None:
            continue
        rows = ",\n\t".join(", ".join("0x%04X" % px for px in icon[y * ICON_PX:(y + 1) * ICON_PX])
                            for y in range(ICON_PX))
        c.append("// icons.png tile %d -- %s\n"
                 "static const uint16_t rom_%d_icon[%d] = {\n\t%s,\n};\n\n"
                 % (slot, name, i, ICON_PX * ICON_PX, rows))
    c.append("const rom_entry_t rom_catalog[ROM_COUNT] = {\n")
    for i, (path, name, slot, _, color, icon) in enumerate(catalog):
        c.append("\t{ \"%s\", rom_%d_data,\n"
                 "\t  (uint32_t)(rom_%d_data_end - rom_%d_data), %d, 0x%04X, %s }, // %s\n"
                 % (name, i, i, i, slot, color or 0,
                    ("rom_%d_icon" % i) if icon else "nullptr",
                    os.path.basename(path)))
    c.append("};\n")

    os.makedirs(out_dir, exist_ok=True)
    write_if_changed(os.path.join(out_dir, "rom_data.S"), "".join(s))
    write_if_changed(os.path.join(out_dir, "rom_data.hpp"), "".join(h))
    write_if_changed(os.path.join(out_dir, "rom_data.cpp"), "".join(c))


def make_dummy(path):
    """Write a minimal valid ROM (32KB, spins at 0x150) -- lets CI build the
    firmware without shipping any real ROM."""
    rom = bytearray(32 * 1024)
    rom[0x100:0x104] = b"\x00\xc3\x50\x01"      # nop; jp 0x0150
    rom[0x134:0x134 + 9] = b"PICODUMMY"          # title
    rom[0x143] = 0x80                            # CGB-enhanced
    rom[0x147] = 0x00                            # ROM only
    rom[0x148] = 0x00                            # 32KB
    rom[0x149] = 0x00                            # no cart RAM
    rom[0x150:0x152] = b"\x18\xfe"               # jr -2 (spin)
    checksum = 0
    for i in range(0x134, 0x14D):
        checksum = (checksum - rom[i] - 1) & 0xFF
    rom[0x14D] = checksum
    with open(path, "wb") as f:
        f.write(rom)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--assets", help="directory holding the .gb/.gbc files")
    ap.add_argument("--out", help="directory to write rom_data.S/.hpp/.cpp into")
    ap.add_argument("--make-dummy", metavar="PATH",
                    help="instead of generating, write a minimal bootable ROM "
                         "to PATH (for CI builds with no real ROMs)")
    args = ap.parse_args()

    if args.make_dummy:
        make_dummy(args.make_dummy)
        return
    if not args.assets or not args.out:
        ap.error("--assets and --out are required (unless --make-dummy)")

    catalog, warnings = build_catalog(os.path.abspath(args.assets))
    for w in warnings:
        sys.stderr.write("gen_rom_data: warning: %s\n" % w)
    emit(catalog, args.out)
    print("gen_rom_data: %d ROM(s), %.2f MB total:" %
          (len(catalog), sum(sz for _, _, _, sz, _, _ in catalog) / 1048576.0))
    for path, name, slot, size, color, icon in catalog:
        tint = "  label 0x%04X" % color if color else ""
        print("  %-24s  save slot %-2d  %5d KB  (%s)%s%s"
              % (name, slot, size // 1024, os.path.basename(path), tint,
                 "  icon" if icon else ""))


if __name__ == "__main__":
    main()
