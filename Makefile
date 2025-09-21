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

.PHONY: all clean clean-all run build-bootloader build-kernel build esp bios test test-all test-bare-metal test-kernel test-panic test-help debug
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

# =============================================================================
# TEST TARGETS
# =============================================================================
# These targets run tests in different environments:
# - bare-metal: Tests run immediately after bootloader handoff (no kernel services)
# - kernel: Tests run after full kernel initialization (with heap, memory mapping, etc.)
# - panic: Tests that verify panic handling works correctly

.PHONY: test test-all test-bare-metal test-kernel test-panic test-help

# Default test target runs bare-metal tests
test: test-bare-metal

# Run all tests
test-all: test-bare-metal test-kernel test-panic

# Show available test targets
test-help:
	@echo "Available test targets:"
	@echo "  test-bare-metal  - Run tests in bare-metal environment (no kernel services)"
	@echo "  test-kernel      - Run tests after kernel initialization (with heap, etc.)"
	@echo "  test-panic       - Run tests that verify panic handling works"
	@echo "  test-all         - Run all test suites"
	@echo "  test             - Run bare-metal tests (default)"

# Bare-metal tests - run immediately after bootloader handoff
test-bare-metal: build-bootloader
	@echo "🧪 Building and running bare-metal tests..."
	@cargo build --package $(KERNEL_PROJECT) --target $(KERNEL_TARGET) --$(PROFILE) --test bare_metal_tests
	@echo "📦 Creating bare-metal test disk image..."
	@$(call create_test_disk,bare_metal_tests,bare_metal_tests)
	@echo "🚀 Running bare-metal tests in QEMU..."
	@$(call run_test_qemu,bare_metal_tests,$(TIMEOUT))
	@echo "✅ Bare-metal tests PASSED"

# Kernel-initialized tests - run after full kernel setup
test-kernel: build-bootloader
	@echo "🧪 Building and running kernel-initialized tests..."
	@cargo build --package $(KERNEL_PROJECT) --target $(KERNEL_TARGET) --$(PROFILE) --test kernel_tests
	@echo "📦 Creating kernel test disk image..."
	@$(call create_test_disk,kernel_tests,kernel_tests)
	@echo "🚀 Running kernel tests in QEMU..."
	@$(call run_test_qemu,kernel_tests,$(TIMEOUT))
	@echo "✅ Kernel tests PASSED"

# Panic tests - verify panic handling works correctly
test-panic: build-bootloader
	@echo "🧪 Building and running panic tests..."
	@cargo build --package $(KERNEL_PROJECT) --target $(KERNEL_TARGET) --$(PROFILE) --test should_panic
	@echo "📦 Creating panic test disk image..."
	@$(call create_test_disk,should_panic,should_panic)
	@echo "🚀 Running panic tests in QEMU..."
	@$(call run_test_qemu,should_panic,$(TIMEOUT))
	@echo "✅ Panic tests PASSED"

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

# Create a test disk image for a specific test
# 
# This function creates a complete EFI bootable disk image containing:
# 1. The UEFI bootloader (theseus_efi.efi)
# 2. The test binary (kernel.efi)
# 3. Proper GPT partition table with EFI System Partition
# 4. FAT32 filesystem
#
# Usage: $(call create_test_disk,test_name,test_binary_name)
# Parameters:
#   $(1) - Test name (used for directory naming)
#   $(2) - Test binary name (used to find the compiled test binary)
define create_test_disk
	# Clean up any existing test directory
	@rm -rf $(ESP_DIR)/test-$(1)
	
	# Create directory structure for EFI boot
	@mkdir -p $(ESP_DIR)/test-$(1)/EFI/BOOT
	
	# Copy the UEFI bootloader (this loads and runs our test)
	@cp $(BOOTLOADER_BUILD_DIR)/theseus_efi.efi $(ESP_DIR)/test-$(1)/EFI/BOOT/BOOTX64.EFI
	
	# Find and copy the compiled test binary
	# The test binary is named like "test_name-hash" in the deps directory
	@find target/$(KERNEL_TARGET)/$(PROFILE)/deps -name "$(2)-*" -type f -executable | head -1 | xargs -I {} cp {} $(ESP_DIR)/test-$(1)/EFI/BOOT/kernel.efi
	
	# Create a 64MB disk image filled with zeros
	@dd if=/dev/zero of=$(ESP_DIR)/test-$(1)/disk.img bs=1M count=64 2>/dev/null
	
	# Create GPT partition table
	@sgdisk --clear $(ESP_DIR)/test-$(1)/disk.img 2>/dev/null || true
	
	# Create EFI System Partition (ESP) from 1MB to 64MB
	@sgdisk --new=1:1M:64M $(ESP_DIR)/test-$(1)/disk.img 2>/dev/null || true
	
	# Set partition type to EFI System Partition
	@sgdisk --typecode=1:C12A7328-F81F-11D2-BA4B-00A0C93EC93B $(ESP_DIR)/test-$(1)/disk.img 2>/dev/null || true
	
	# Format the partition as FAT32 with label "ESP"
	@mkfs.fat -F32 -n ESP $(ESP_DIR)/test-$(1)/disk.img 2>/dev/null
	
	# Create EFI directory structure on the filesystem
	@mmd -i $(ESP_DIR)/test-$(1)/disk.img ::EFI 2>/dev/null || true
	@mmd -i $(ESP_DIR)/test-$(1)/disk.img ::EFI/BOOT 2>/dev/null || true
	
	# Copy bootloader to the EFI filesystem
	@mcopy -i $(ESP_DIR)/test-$(1)/disk.img -s $(BOOTLOADER_BUILD_DIR)/theseus_efi.efi ::EFI/BOOT/BOOTX64.EFI 2>/dev/null || true
	
	# Copy test binary to the EFI filesystem as kernel.efi
	@find target/$(KERNEL_TARGET)/$(PROFILE)/deps -name "$(2)-*" -type f -executable | head -1 | xargs -I {} mcopy -i $(ESP_DIR)/test-$(1)/disk.img {} ::kernel.efi 2>/dev/null || true
endef

# Run a test in QEMU with timeout and result checking
# 
# This function runs a test in QEMU with the following features:
# 1. Automatic timeout to prevent hanging
# 2. QEMU debug port output capture
# 3. isa-debug-exit device for automatic pass/fail detection
# 4. Proper error handling and reporting
# 5. Log file generation for debugging
#
# Usage: $(call run_test_qemu,test_name,timeout_seconds)
# Parameters:
#   $(1) - Test name (used for log file naming)
#   $(2) - Timeout in seconds
define run_test_qemu
	# Run QEMU with the test disk image and capture output
	# Key QEMU options:
	# - timeout: Prevent hanging tests from running forever
	# - machine q35: Modern PC emulation
	# - accel=kvm:tcg: Use KVM if available, fallback to TCG
	# - cpu max: Use all available CPU features
	# - m 1G: 1GB of RAM
	# - isa-debug-exit: Device for automatic test result detection
	# - isa-debugcon: Device for QEMU debug port output
	# - display none: Run headless
	# - no-reboot: Don't restart on exit
	@timeout $(2)s qemu-system-x86_64 \
		-machine q35,accel=kvm:tcg \
		-cpu max \
		-m 1G \
		-drive if=pflash,format=raw,readonly=on,file="$(OVMF_CODE)" \
		-drive if=pflash,format=raw,file="$(OVMF_VARS_RW)" \
		-device isa-debug-exit,iobase=0xf4,iosize=0x04 \
		-device isa-debugcon,chardev=debugcon \
		-chardev stdio,id=debugcon \
		-display none \
		-serial null \
		-monitor none \
		-drive format=raw,file="$(ESP_DIR)/test-$(1)/disk.img" \
		-nic none \
		-no-reboot \
		2>&1 | tee /tmp/$(1)_test_output.log
	
	# Check the exit code from QEMU
	# Exit code 0 means the test passed (isa-debug-exit with code 0)
	# Any other exit code means the test failed or timed out
	@if [ $$? -eq 0 ]; then \
		echo "✅ $(1) tests PASSED"; \
	else \
		echo "❌ $(1) tests FAILED"; \
		echo "Last 20 lines of output:"; \
		tail -20 /tmp/$(1)_test_output.log; \
		exit 1; \
	fi
endef


