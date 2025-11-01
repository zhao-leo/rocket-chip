base_dir=$(abspath ./)

CHISEL_VERSION=3.6.0
MODEL ?= TestHarness
CONFIG ?= DefaultConfig
PROJECT ?= freechips.rocketchip.system
CFG_PROJECT ?= $(PROJECT)
CONFIG_FULL ?= $(CFG_PROJECT).$(CONFIG)
MILL ?= mill

verilog:
	cd $(base_dir) && $(MILL) emulator[freechips.rocketchip.system.TestHarness,$(CONFIG_FULL)].mfccompiler.compile

clean:
	rm -rf out/
