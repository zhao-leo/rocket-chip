export ROCKET_TOOLS_PATH=$(readlink -f rocket-tools)
export RISCV=$ROCKET_TOOLS_PATH/riscv-gnu-toolchain
echo "RISCV: $RISCV"
