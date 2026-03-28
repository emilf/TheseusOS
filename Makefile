BOOTLOADER_PROJECT := theseus-bootloader
KERNEL_PROJECT := theseus-kernel
BOOTLOADER_TARGET := x86_64-unknown-uefi-dwarf
BOOTLOADER_TARGET_SPEC := $(BOOTLOADER_TARGET).json
KERNEL_TARGET := x86_64-unknown-none
PROFILE ?= release
BOOTLOADER_CARGO_FLAGS := -Z build-std=core,alloc -Z json-target-spec
BOOTLOADER_BUILD_DIR := target/$(BOOTLOADER_TARGET)/$(PROFILE)
EFI_DIR := build/EFI/BOOT
ESP_DIR := build
EFI_BIN := $(BOOTLOADER_BUILD_DIR)/theseus_efi.efi
EFI_OUTPUT := $(EFI_DIR)/BOOTX64.EFI
SYMBOL_OUTPUT := $(ESP_DIR)/BOOTX64.SYM
OVMF_DIR := OVMF
OVMF_CODE := $(OVMF_DIR)/OVMF_CODE.fd
OVMF_VARS_ORIG := $(OVMF_DIR)/OVMF_VARS.fd
OVMF_VARS_RW := $(ESP_DIR)/OVMF_VARS.fd
OBJCOPY ?= objcopy

ifeq ($(strip $(shell command -v $(OBJCOPY) >/dev/null 2>&1 && echo ok)),)
$(error objcopy not found. Install GNU binutils (provides objcopy) or set OBJCOPY to a compatible tool.)
endif

RUSTFLAGS += -C debuginfo=2
RUSTFLAGS += -C split-debuginfo=off
export RUSTFLAGS

# Map PROFILE to the correct cargo flag. Cargo uses `--release` for release
# builds and no flag for debug builds. Set `PROFILE=debug` to build debug.
ifeq ($(PROFILE),release)
CARGO_PROFILE_FLAG := --release
else
CARGO_PROFILE_FLAG :=
endif

.PHONY: all clean clean-all run build-bootloader build esp bios debug help debug-build
all: build esp bios

build: build-bootloader

build-bootloader:
	cargo build $(BOOTLOADER_CARGO_FLAGS) --package $(BOOTLOADER_PROJECT) --target $(BOOTLOADER_TARGET_SPEC) $(CARGO_PROFILE_FLAG) $(if $(FEATURES),--features $(FEATURES),)

esp: $(EFI_OUTPUT) $(SYMBOL_OUTPUT)

$(EFI_OUTPUT): $(EFI_BIN)
	@echo "Creating EFI System Partition..."
	@rm -rf $(ESP_DIR)
	@mkdir -p $(EFI_DIR)
	@cp $(EFI_BIN) $(EFI_OUTPUT)
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
	@echo "✓ Created GPT disk image with EFI System Partition"

$(SYMBOL_OUTPUT): $(EFI_OUTPUT)
	@mkdir -p $(dir $(SYMBOL_OUTPUT))
	@echo "Generating ELF symbol file for GDB..."
	@$(OBJCOPY) --input-target=pei-x86-64 --output-target=elf64-x86-64 $(EFI_BIN) $(SYMBOL_OUTPUT)
	@printf '\177ELF' | cmp -s -n 4 - $(SYMBOL_OUTPUT) || { \
		echo "Error: failed to generate ELF symbols with $(OBJCOPY)." >&2; \
		echo "Install GNU binutils objcopy or set OBJCOPY=<path-to-gnu-objcopy>." >&2; \
		rm -f $(SYMBOL_OUTPUT); \
		exit 1; \
	}
	@echo "✓ Generated ELF symbol file for GDB"

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

.PHONY: debug
debug: all
	@echo "Starting QEMU paused with GDB on :1234 and monitor on 127.0.0.1:55555"
	QEMU_OPTS="-S -s" ./startQemu.sh headless

# Automated GDB session via pexpect (no manual address copying, no probe run).
# Uses the debug mailbox (physical 0x7000) — efi_main writes its own runtime
# address there on entry; GDB watches for it via a hardware watchpoint.
#
# Requires: pip install --break-system-packages pexpect
# Requires: tmux session named 'theseus' (created automatically if absent)
#
# Full interactive session (default):
#   make debug-auto
#
# Non-interactive smoke-test (CI-friendly, exits after breakpoint check):
#   make debug-auto-ci
.PHONY: debug-auto debug-auto-ci
debug-auto: all
	@echo "Starting automated GDB session (mailbox watchpoint + pexpect)..."
	python3 scripts/gdb-auto.py --tmux theseus

debug-auto-ci: all
	@echo "Starting non-interactive GDB breakpoint smoke-test..."
	python3 scripts/gdb-auto.py --tmux theseus \
		--no-interactive --timeout-boot 180

# Print a short help message describing common targets and how to set PROFILE
.PHONY: help
help:
	@echo "Usage: make [target] [VARIABLE=value]"
	@echo "Common targets: build, esp, run, run-headed, debug, debug-build"
	@echo "Default PROFILE is 'release'. To build debug artifacts and include them in the disk image, set PROFILE=debug:" \
		&& echo "  make PROFILE=debug build esp" \
		&& echo "Or to build and start QEMU paused for GDB:" \
		&& echo "  make PROFILE=debug debug" \
		&& echo "" \
		&& echo "FEATURES example (pass cargo features):" \
		&& echo "  make FEATURES=foo build"

# Convenience target that runs a debug build and creates the ESP disk image
.PHONY: debug-build
debug-build:
	@echo "Running debug build and creating ESP image..."
	$(MAKE) PROFILE=debug build esp

