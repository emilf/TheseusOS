PROJECT := hobbyos_efi
TARGET := x86_64-unknown-uefi
PROFILE := release
BUILD_DIR := target/$(TARGET)/$(PROFILE)
EFI_DIR := build/EFI/BOOT
ESP_DIR := build
EFI_BIN := $(BUILD_DIR)/$(PROJECT).efi
EFI_OUTPUT := $(EFI_DIR)/BOOTX64.EFI

.PHONY: all clean run build esp

all: build esp

build:
	cargo build --$(PROFILE)

esp: $(EFI_OUTPUT)

$(EFI_OUTPUT): $(EFI_BIN)
	rm -rf $(ESP_DIR)
	mkdir -p $(EFI_DIR)
	cp $(EFI_BIN) $(EFI_OUTPUT)

clean:
	cargo clean
	rm -rf $(ESP_DIR)

run: all
	./startQemu.sh


