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

.PHONY: debug
debug: all
	@echo "Starting QEMU paused with GDB on :1234 and monitor on 127.0.0.1:55555"
	OVMF_DIR=$$(pwd)/OVMF OVMF_CODE=$$(pwd)/OVMF/OVMF_CODE.fd OVMF_VARS=$$(pwd)/OVMF/OVMF_VARS.fd \
	QEMU_OPTS="-S -s" ./startQemu.sh


