base_dir=$(abspath ./)

CHISEL_VERSION=3.6.0
MODEL ?= TestHarness
CONFIG ?= DefaultConfig
PROJECT ?= freechips.rocketchip.system
CFG_PROJECT ?= $(PROJECT)
CONFIG_FULL ?= $(CFG_PROJECT).$(CONFIG)
MILL ?= mill

# Default verilog generation (for simulation)
verilog:
	@echo -e "\nGenerating Verilog..."
	@cd $(base_dir) && $(MILL) emulator[freechips.rocketchip.system.$(MODEL),$(CONFIG_FULL)].mfccompiler.compile

clean:
	@echo -e "\nCleaning..."
	@rm -rf out
