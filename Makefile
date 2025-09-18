BOOTLOADER_PROJECT := theseus-bootloader
KERNEL_PROJECT := theseus-kernel
BOOTLOADER_TARGET := x86_64-unknown-uefi
KERNEL_TARGET := x86_64-unknown-none
PROFILE := release
BOOTLOADER_BUILD_DIR := target/$(BOOTLOADER_TARGET)/$(PROFILE)
KERNEL_BUILD_DIR := target/$(KERNEL_TARGET)/$(PROFILE)
EFI_DIR := build/EFI/BOOT
ESP_DIR := build
EFI_BIN := $(BOOTLOADER_BUILD_DIR)/theseus_efi.efi
KERNEL_BIN := $(KERNEL_BUILD_DIR)/kernel
EFI_OUTPUT := $(EFI_DIR)/BOOTX64.EFI
OVMF_DIR := OVMF
OVMF_CODE := $(OVMF_DIR)/OVMF_CODE.fd
OVMF_VARS_ORIG := $(OVMF_DIR)/OVMF_VARS.fd
OVMF_VARS_RW := $(ESP_DIR)/OVMF_VARS.fd

.PHONY: all clean clean-all run build-bootloader build-kernel build esp bios
TIMEOUT ?= 20

all: build esp bios

build: build-bootloader build-kernel

build-bootloader:
	cargo build --package $(BOOTLOADER_PROJECT) --target $(BOOTLOADER_TARGET) --$(PROFILE) $(if $(FEATURES),--features $(FEATURES),)

build-kernel:
	cargo build --package $(KERNEL_PROJECT) --target $(KERNEL_TARGET) --$(PROFILE) $(if $(FEATURES),--features $(FEATURES),)

esp: $(EFI_OUTPUT)

$(EFI_OUTPUT): $(EFI_BIN) $(KERNEL_BIN)
	@echo "Creating EFI System Partition..."
	@rm -rf $(ESP_DIR)
	@mkdir -p $(EFI_DIR)
	@cp $(EFI_BIN) $(EFI_OUTPUT)
	@cp $(KERNEL_BIN) $(EFI_DIR)/kernel.efi
	@# Create a proper GPT disk image with EFI System Partition
	@echo "Creating GPT disk image with EFI System Partition..."
	@dd if=/dev/zero of=$(ESP_DIR)/disk.img bs=1M count=64 2>/dev/null
	@# Create GPT partition table and ESP partition using sgdisk
	@sgdisk --clear $(ESP_DIR)/disk.img 2>/dev/null || true
	@sgdisk --new=1:1M:64M $(ESP_DIR)/disk.img 2>/dev/null || true
	@sgdisk --typecode=1:C12A7328-F81F-11D2-BA4B-00A0C93EC93B $(ESP_DIR)/disk.img 2>/dev/null || true
	@# Format the partition as FAT32
	@mkfs.fat -F32 -n ESP $(ESP_DIR)/disk.img 2>/dev/null
	@# Copy files using mcopy (part of mtools, no mounting required)
	@# Create directory structure first
	@mmd -i $(ESP_DIR)/disk.img ::EFI 2>/dev/null || true
	@mmd -i $(ESP_DIR)/disk.img ::EFI/BOOT 2>/dev/null || true
	@# Copy the bootloader and kernel
	@mcopy -i $(ESP_DIR)/disk.img -s $(EFI_BIN) ::EFI/BOOT/BOOTX64.EFI 2>/dev/null || true
	@mcopy -i $(ESP_DIR)/disk.img -s $(KERNEL_BIN) ::kernel.efi 2>/dev/null || true
	@echo "✓ Created GPT disk image with EFI System Partition"

# Automatically copy BIOS files if needed
bios: $(OVMF_CODE) $(OVMF_VARS_ORIG) $(OVMF_VARS_RW)

# Dependencies - BIOS files run after ESP is created
$(OVMF_CODE): $(EFI_OUTPUT)
$(OVMF_VARS_ORIG): $(EFI_OUTPUT)
$(OVMF_VARS_RW): $(OVMF_VARS_ORIG)

$(OVMF_CODE):
	@echo "Setting up OVMF_CODE..."
	@mkdir -p $(OVMF_DIR)
	@# Try to find and copy OVMF_CODE file from system locations
	@for base in \
		/usr/share/edk2-ovmf/x64 \
		/usr/share/edk2/x64 \
		/usr/share/OVMF; do \
		if [ -f "$$base/OVMF_CODE.4m.fd" ]; then \
			echo "Found OVMF_CODE.4m.fd in $$base"; \
			cp -f "$$base/OVMF_CODE.4m.fd" "$(OVMF_CODE)"; \
			break; \
		elif [ -f "$$base/OVMF_CODE.fd" ]; then \
			echo "Found OVMF_CODE.fd in $$base"; \
			cp -f "$$base/OVMF_CODE.fd" "$(OVMF_CODE)"; \
			break; \
		fi; \
	done
	@if [ ! -f "$(OVMF_CODE)" ]; then \
		echo "Error: Could not find OVMF_CODE file in system locations" >&2; \
		echo "Please install edk2-ovmf package" >&2; \
		exit 1; \
	fi

$(OVMF_VARS_ORIG):
	@echo "Setting up OVMF_VARS original..."
	@mkdir -p $(OVMF_DIR)
	@# Try to find and copy OVMF_VARS file from system locations
	@for base in \
		/usr/share/edk2-ovmf/x64 \
		/usr/share/edk2/x64 \
		/usr/share/OVMF; do \
		if [ -f "$$base/OVMF_VARS.4m.fd" ]; then \
			echo "Found OVMF_VARS.4m.fd in $$base"; \
			cp -f "$$base/OVMF_VARS.4m.fd" "$(OVMF_VARS_ORIG)"; \
			break; \
		elif [ -f "$$base/OVMF_VARS.fd" ]; then \
			echo "Found OVMF_VARS.fd in $$base"; \
			cp -f "$$base/OVMF_VARS.fd" "$(OVMF_VARS_ORIG)"; \
			break; \
		fi; \
	done
	@if [ ! -f "$(OVMF_VARS_ORIG)" ]; then \
		echo "Error: Could not find OVMF_VARS file in system locations" >&2; \
		echo "Please install edk2-ovmf package" >&2; \
		exit 1; \
	fi

$(OVMF_VARS_RW): $(OVMF_VARS_ORIG)
	@echo "Setting up OVMF_VARS writable copy..."
	@mkdir -p $(ESP_DIR)
	@cp -f "$(OVMF_VARS_ORIG)" "$(OVMF_VARS_RW)"

clean:
	cargo clean
	rm -rf $(ESP_DIR)

clean-all: clean
	rm -rf $(OVMF_DIR)

# Run targets
run: all
	@echo "Starting QEMU in headless mode..."
	./startQemu.sh headless

run-headed: all
	@echo "Starting QEMU in headed mode..."
	./startQemu.sh headed

run-test: all
	@echo "Starting QEMU with $(TIMEOUT)s timeout for testing..."
	./startQemu.sh headless $(TIMEOUT)

.PHONY: debug
debug: all
	@echo "Starting QEMU paused with GDB on :1234 and monitor on 127.0.0.1:55555"
	QEMU_OPTS="-S -s" ./startQemu.sh headless


