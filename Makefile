# Convenience wrapper so "drop ROMs in assets/, run make" is the whole build.
# Real build logic lives in CMakeLists.txt; PICO_SDK_PATH is picked up from
# the environment if set (otherwise the SDK is fetched from git on first
# configure).

BUILD_DIR ?= build

.PHONY: all clean

all:
	cmake -S . -B $(BUILD_DIR)
	cmake --build $(BUILD_DIR) --parallel
	@echo
	@echo "Built $(BUILD_DIR)/PicoCrystal.uf2 -- hold X while switching the"
	@echo "PicoSystem on, then copy it onto the RPI-RP2 USB drive that appears."

clean:
	rm -rf $(BUILD_DIR)
