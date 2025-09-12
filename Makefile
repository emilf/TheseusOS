PROJECT := hobbyos_efi
TARGET := x86_64-unknown-uefi
PROFILE := release
BUILD_DIR := target/$(TARGET)/$(PROFILE)
EFI_DIR := build/EFI/BOOT
ESP_DIR := build
EFI_BIN := $(BUILD_DIR)/$(PROJECT).efi
EFI_OUTPUT := $(EFI_DIR)/BOOTX64.EFI
OVMF_DIR := OVMF
OVMF_CODE := $(OVMF_DIR)/OVMF_CODE.fd
OVMF_VARS_ORIG := $(OVMF_DIR)/OVMF_VARS.fd
OVMF_VARS_RW := $(ESP_DIR)/OVMF_VARS.fd

.PHONY: all clean clean-all run build esp bios

all: build esp bios

build:
	cargo build --$(PROFILE)

esp: $(EFI_OUTPUT)

$(EFI_OUTPUT): $(EFI_BIN)
	rm -rf $(ESP_DIR)
	mkdir -p $(EFI_DIR)
	cp $(EFI_BIN) $(EFI_OUTPUT)

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
	@echo "Starting QEMU with 20s timeout for testing..."
	./startQemu.sh headless 20

.PHONY: debug
debug: all
	@echo "Starting QEMU paused with GDB on :1234 and monitor on 127.0.0.1:55555"
	QEMU_OPTS="-S -s" ./startQemu.sh headless


