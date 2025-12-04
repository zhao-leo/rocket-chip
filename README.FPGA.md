# FPGA Rocket Chip 部署指南

本文档描述了 FPGA 版本 Rocket Chip 的内存映射、外设配置以及完整的 Linux 构建与烧写流程。

## 系统架构概述

- **处理器核心**: Big Rocket Core (RV64IMAFDC)
- **RoCC 加速器**: 已启用 (RoccExample)
- **调试模块**: 已禁用 (仅 UART 输出)
- **输入时钟**: 200 MHz 差分时钟 (LVDS)
- **系统时钟**: 50 MHz (通过 MMCME2_BASE 分频)
- **时钟原语**: IBUFDS + MMCME2_BASE + BUFG

## 内存映射 (Memory Map)

### 完整地址空间

| 区域 | 起始地址 | 结束地址 | 大小 | 描述 |
|------|----------|----------|------|------|
| Debug | `0x0000_0000` | `0x0000_0FFF` | 4 KB | Debug 模块 (已禁用) |
| CLINT | `0x0200_0000` | `0x0200_FFFF` | 64 KB | Core Local Interruptor |
| PLIC | `0x0C00_0000` | `0x0FFF_FFFF` | 64 MB | Platform-Level Interrupt Controller |
| UART | `0x1000_0000` | `0x1000_0FFF` | 4 KB | SiFive UART 控制器 |
| Boot ROM | `0x1000_1000` | `0x1001_0FFF` | 64 KB | 启动 ROM |
| RAM | `0x8000_0000` | `0xBFFF_FFFF` | 1 GB | 主存储器 |

### 关键地址汇总

```
CLINT (mtime/mtimecmp):  0x0200_0000
PLIC:                    0x0C00_0000
UART:                    0x1000_0000
Memory Base:             0x8000_0000
Memory Size:             0x4000_0000 (1GB)
```

## UART 配置详解

### 基地址与寄存器

UART 基地址: `0x1000_0000`

| 偏移 | 寄存器名 | 读/写 | 描述 |
|------|----------|-------|------|
| `0x00` | txdata | RW | 发送数据 [7:0], full 标志 [31] |
| `0x04` | rxdata | RO | 接收数据 [7:0], empty 标志 [31] |
| `0x08` | txctrl | RW | 发送控制: txen[0], nstop[1], txcnt[18:16] |
| `0x0C` | rxctrl | RW | 接收控制: rxen[0], rxcnt[18:16] |
| `0x10` | ie | RW | 中断使能: txwm[0], rxwm[1] |
| `0x14` | ip | RO | 中断挂起: txwm[0], rxwm[1] |
| `0x18` | div | RW | 波特率分频器 [15:0] |

### 默认配置

- **波特率**: 115200 bps
- **数据位**: 8
- **停止位**: 1
- **校验**: 无
- **流控**: 无 (CTS/RTS 已 tie-off)

### 波特率计算公式

```
div = (系统时钟频率 / 目标波特率) - 1
```

50 MHz 系统时钟下:
```
div = (50000000 / 115200) - 1 = 433 (0x1B1)
```

### UART 中断号

- **PLIC 中断号**: 1

## 中断配置

### CLINT (Core Local Interruptor)

| 偏移 | 寄存器 | 描述 |
|------|--------|------|
| `0x0000` | msip[0] | Machine Software Interrupt Pending |
| `0x4000` | mtimecmp[0] | Machine Timer Compare (64-bit) |
| `0xBFF8` | mtime | Machine Time (64-bit) |

- **Timebase 频率**: 1 MHz

### PLIC (Platform-Level Interrupt Controller)

- **最大中断源数**: 2 (`WithNExtTopInterrupts(2)`)
- **中断 1**: UART

## 顶层端口定义

### FPGARocketTop

| 端口名 | 方向 | 位宽 | 描述 |
|--------|------|------|------|
| `clock_p` | Input | 1 | 200 MHz 差分时钟正端 |
| `clock_n` | Input | 1 | 200 MHz 差分时钟负端 |
| `reset` | Input | 1 | 复位信号 (高有效) |
| `uart_tx` | Output | 1 | UART 发送数据 |
| `uart_rx` | Input | 1 | UART 接收数据 |

---

# 完整 Linux 启动方案

## 总体流程

```
┌─────────────────────────────────────────────────────────────────────┐
│  1. 生成 Verilog → 2. Vivado 综合 → 3. 生成 Bitstream → 4. 烧录 FPGA│
└─────────────────────────────────────────────────────────────────────┘
                                  ↓
┌─────────────────────────────────────────────────────────────────────┐
│  5. 编译 Linux 内核 → 6. 编译 DTB → 7. 编译 Rootfs → 8. 打包 Image  │
└─────────────────────────────────────────────────────────────────────┘
                                  ↓
┌─────────────────────────────────────────────────────────────────────┐
│  9. 通过 Vivado 将 Image 加载到 RAM → 10. 释放复位 → 11. Linux 启动 │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 第一步: 生成 Verilog

```bash
cd rocket-chip
nix develop -c make verilog MODEL=FPGARocketTop  CONFIG=DefaultFPGARocketConfig
```

生成的文件位于 `out/emulator/freechips.rocketchip.system.FPGARocketTop/freechips.rocketchip.system.DefaultFPGARocketConfig/mfccompiler/compile.dest` 目录。

---

## 第二步: Vivado 工程配置

### 创建 Vivado 工程

1. 打开 Vivado，创建新工程
2. 添加生成的 Verilog 文件
3. 设置顶层模块为 `FPGARocketTop`

### XDC 约束文件示例

根据你的 FPGA 板卡修改引脚分配：

```tcl
# ============================================================================
# 时钟约束 (200 MHz 差分输入)
# ============================================================================
set_property PACKAGE_PIN <your_clk_p_pin> [get_ports clock_p]
set_property PACKAGE_PIN <your_clk_n_pin> [get_ports clock_n]
set_property IOSTANDARD LVDS [get_ports clock_p]
set_property IOSTANDARD LVDS [get_ports clock_n]

# 输入时钟约束 (200 MHz = 5ns)
create_clock -period 5.000 -name sys_clk_200 [get_ports clock_p]

# 生成的时钟约束 (50 MHz)
# Vivado 会自动推断 MMCM 输出时钟，但可以显式约束
# create_generated_clock -name sys_clk_50 -source [get_pins clkGen/mmcm/CLKIN1] \
#     -divide_by 4 [get_pins clkGen/mmcm/CLKOUT0]

# ============================================================================
# 复位
# ============================================================================
set_property PACKAGE_PIN <your_reset_pin> [get_ports reset]
set_property IOSTANDARD LVCMOS33 [get_ports reset]

# ============================================================================
# UART
# ============================================================================
set_property PACKAGE_PIN <your_uart_tx_pin> [get_ports uart_tx]
set_property PACKAGE_PIN <your_uart_rx_pin> [get_ports uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]

# ============================================================================
# 时序例外 (异步复位)
# ============================================================================
set_false_path -from [get_ports reset]
```

### 综合与实现

```tcl
# 在 Vivado Tcl Console 中执行
synth_design -top FPGARocketTop -part <your_fpga_part>
opt_design
place_design
route_design
write_bitstream -force fpga_rocket.bit
```

---

## 第三步: 构建 Linux 启动镜像

### 3.1 准备工具链

安装 RISC-V 工具链:

```bash
# Ubuntu/Debian
sudo apt-get install gcc-riscv64-linux-gnu

# 或从源码编译
git clone https://github.com/riscv-collab/riscv-gnu-toolchain
# 后续内容参考原仓库文档
```

### 3.2 编译 OpenSBI (可选的 Bootloader)

OpenSBI 提供 M-mode 运行时服务:

```bash
git clone https://github.com/riscv-software-src/opensbi.git
cd opensbi

make CROSS_COMPILE=riscv64-linux-gnu- \
     PLATFORM=generic \
     FW_TEXT_START=0x80000000 \
     FW_JUMP_ADDR=0x80200000 \
     FW_JUMP_FDT_ADDR=0x82000000
```

生成文件: `build/platform/generic/firmware/fw_jump.bin`

### 3.3 编译 Linux 内核

```bash
git clone --depth 1 https://github.com/torvalds/linux.git
cd linux

# 配置内核
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- defconfig

# 自定义配置 (重要!)
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- menuconfig
```

**必须启用的内核选项:**

```
Device Drivers --->
    Character devices --->
        Serial drivers --->
            [*] SiFive UART support
            [*]   Console on SiFive UART

General setup --->
    [*] Initial RAM filesystem and RAM disk (initramfs/initrd) support
        () Initramfs source file(s)  # 留空，后面手动指定

# 如果内存较小，启用压缩
General setup --->
    Kernel compression mode (XZ)  --->

# 关闭不需要的功能以减小内核体积
Networking support --->
    [ ] Networking support  # 如不需要网络可关闭
```

**编译内核:**

```bash
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- -j$(nproc)
```

生成文件: `arch/riscv/boot/Image`

### 3.4 创建设备树 (DTS)

创建文件 `rocket-fpga.dts`:

```dts
/dts-v1/;

/ {
    #address-cells = <2>;
    #size-cells = <2>;
    compatible = "freechips,rocketchip-fpga";
    model = "Rocket Chip FPGA";

    chosen {
        bootargs = "earlycon=sifive,0x10000000 console=ttySIF0,115200 root=/dev/ram0 rw";
        stdout-path = "serial0:115200n8";
    };

    aliases {
        serial0 = &uart0;
    };

    cpus {
        #address-cells = <1>;
        #size-cells = <0>;
        timebase-frequency = <1000000>;  // 1 MHz

        cpu0: cpu@0 {
            device_type = "cpu";
            reg = <0>;
            compatible = "sifive,rocket0", "riscv";
            riscv,isa = "rv64imafdc";
            mmu-type = "riscv,sv39";
            clock-frequency = <50000000>;  // 50 MHz

            cpu0_intc: interrupt-controller {
                #interrupt-cells = <1>;
                compatible = "riscv,cpu-intc";
                interrupt-controller;
            };
        };
    };

    memory@80000000 {
        device_type = "memory";
        reg = <0x0 0x80000000 0x0 0x40000000>;  // 1GB RAM
    };

    soc {
        #address-cells = <2>;
        #size-cells = <2>;
        compatible = "simple-bus";
        ranges;

        clint0: clint@2000000 {
            compatible = "sifive,clint0", "riscv,clint0";
            reg = <0x0 0x2000000 0x0 0x10000>;
            interrupts-extended = <&cpu0_intc 3>, <&cpu0_intc 7>;
        };

        plic0: interrupt-controller@c000000 {
            compatible = "sifive,plic-1.0.0", "riscv,plic0";
            reg = <0x0 0xc000000 0x0 0x4000000>;
            #address-cells = <0>;
            #interrupt-cells = <1>;
            interrupt-controller;
            interrupts-extended = <&cpu0_intc 11>, <&cpu0_intc 9>;
            riscv,ndev = <2>;
        };

        uart0: serial@10000000 {
            compatible = "sifive,uart0";
            reg = <0x0 0x10000000 0x0 0x1000>;
            interrupt-parent = <&plic0>;
            interrupts = <1>;
            clocks = <&sysclk>;
            clock-frequency = <50000000>;
        };

        sysclk: sysclk {
            compatible = "fixed-clock";
            #clock-cells = <0>;
            clock-frequency = <50000000>;
            clock-output-names = "sysclk";
        };
    };
};
```

**编译 DTB:**

```bash
dtc -I dts -O dtb -o rocket-fpga.dtb rocket-fpga.dts
```

### 3.5 创建 Initramfs (根文件系统)

#### 方案 A: 使用 BusyBox (最小化)

```bash
# 下载 BusyBox
git clone --depth 1 https://github.com/mirror/busybox.git
cd busybox

# fix gcc14 linter
sed -i 's/main/int main/g' scripts/kconfig/lxdialog/check-lxdialog.sh

# 配置
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- defconfig
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- menuconfig

# Fix Bugs
rm -f networking/tc.c
sed -i '/#if ENABLE_SHA1_HWACCEL$/{N;N;s/#if ENABLE_SHA1_HWACCEL\n\t || ctx->process_block == sha1_process_block64_shaNI\n#endif/#if ENABLE_SHA1_HWACCEL\n# if defined(__GNUC__) \&\& (defined(__i386__) \|\| defined(__x86_64__))\n\t || ctx->process_block == sha1_process_block64_shaNI\n# endif\n#endif/}' libbb/hash_md5_sha.c

```

**BusyBox 配置:**
```
Settings --->
    [*] Build static binary (no shared libs)
```

```bash
# 编译
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- -j$(nproc)
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- install

# 创建 initramfs 目录结构
mkdir -p initramfs/{bin,sbin,etc,proc,sys,dev,tmp,root}
cp -a _install/* initramfs/

# 创建 init 脚本
cat > initramfs/init << 'EOF'
#!/bin/sh
mount -t proc none /proc
mount -t sysfs none /sys
mount -t devtmpfs none /dev

echo "Welcome to Rocket Chip Linux!"
echo "UART: 0x10000000, 115200 baud"
echo ""

exec /bin/sh
EOF

chmod +x initramfs/init

# 创建设备节点 (如果没有 devtmpfs)
cd initramfs/dev
sudo mknod -m 666 null c 1 3
sudo mknod -m 666 zero c 1 5
sudo mknod -m 666 tty c 5 0
sudo mknod -m 666 console c 5 1
sudo mknod -m 666 ttySIF0 c 4 64
cd ../..

# 打包 initramfs
cd initramfs
find . | cpio -o -H newc | gzip > ../initramfs.cpio.gz
cd ..
```

#### 方案 B: 使用 Buildroot (完整系统)

```bash
git clone https://github.com/buildroot/buildroot.git
cd buildroot

# 配置
make menuconfig
```

**Buildroot 配置:**
```
Target options --->
    Target Architecture: RISCV
    Target Architecture Variant: 64-bit

Build options --->
    (riscv64-linux-gnu-) Host prefix

Toolchain --->
    C library: musl

System configuration --->
    (rocket) System hostname
    (Welcome to Rocket Chip) System banner
    /dev management: Dynamic using devtmpfs only
    [*] Enable root login with password
    ()  Root password
    Init system: BusyBox
    /bin/sh: busybox' default shell
    [*] Run a getty (login prompt) after boot
        (ttySIF0) TTY port
        (115200) Baudrate

Target packages --->
    根据需要选择软件包
```

```bash
make -j$(nproc)
```

生成文件: `output/images/rootfs.cpio.gz`

---

## 第四步: 创建启动镜像

### 内存布局

```
┌────────────────────────────────────────────────────────────────┐
│  地址范围                    │  内容                           │
├────────────────────────────────────────────────────────────────┤
│  0x8000_0000 - 0x8001_FFFF  │  OpenSBI (fw_jump.bin) ~128KB    │
│  0x8020_0000 - 0x80FF_FFFF  │  Linux Kernel (Image) ~14MB      │
│  0x8200_0000 - 0x8200_FFFF  │  Device Tree (DTB) ~64KB         │
│  0x8300_0000 - ...          │  Initramfs (rootfs) ~varies      │
│  ...                        │  空闲内存 (Linux 使用)           │
│  0xBFFF_FFFF                │  RAM 结束                        │
└────────────────────────────────────────────────────────────────┘
```

### 创建合并的二进制镜像

创建脚本 `create_image.sh`:

```bash
#!/bin/bash

# 配置路径
OPENSBI_BIN="opensbi/build/platform/generic/firmware/fw_jump.bin"
KERNEL_IMG="linux/arch/riscv/boot/Image"
DTB_FILE="rocket-fpga.dtb"
INITRAMFS="initramfs.cpio.gz"
OUTPUT="rocket_linux.bin"

# 基地址 (相对于 0x80000000)
OPENSBI_OFFSET=0x00000000      # 0x8000_0000
KERNEL_OFFSET=0x00200000       # 0x8020_0000
DTB_OFFSET=0x02000000          # 0x8200_0000
INITRAMFS_OFFSET=0x03000000    # 0x8300_0000

# 镜像总大小 (根据需要调整)
IMAGE_SIZE=$((256 * 1024 * 1024))  # 256MB

# 创建空白镜像
dd if=/dev/zero of=$OUTPUT bs=1M count=256

# 写入 OpenSBI
dd if=$OPENSBI_BIN of=$OUTPUT bs=1 seek=$OPENSBI_OFFSET conv=notrunc

# 写入 Linux Kernel
dd if=$KERNEL_IMG of=$OUTPUT bs=1 seek=$KERNEL_OFFSET conv=notrunc

# 写入 DTB
dd if=$DTB_FILE of=$OUTPUT bs=1 seek=$DTB_OFFSET conv=notrunc

# 写入 Initramfs
dd if=$INITRAMFS of=$OUTPUT bs=1 seek=$INITRAMFS_OFFSET conv=notrunc

echo "Image created: $OUTPUT"
echo "Load address: 0x80000000"
ls -lh $OUTPUT
```

```bash
chmod +x create_image.sh
./create_image.sh
```

---

## 第五步: 通过 Vivado 加载镜像到 RAM

### 方法 1: 使用 Vivado Tcl 脚本

需要在设计中添加 AXI Debug Bridge 或使用 JTAG-to-AXI Master IP。

#### 添加 JTAG-to-AXI Master IP

在 Vivado Block Design 中:
1. 添加 `JTAG to AXI Master` IP
2. 连接到系统 AXI 总线
3. 重新综合生成 bitstream

#### Tcl 脚本加载镜像

创建 `load_image.tcl`:

```tcl
# load_image.tcl - 通过 JTAG-to-AXI 加载 Linux 镜像到 RAM

# 配置
set image_file "rocket_linux.bin"
set base_addr 0x80000000

# 打开硬件连接
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

# 获取设备
set device [lindex [get_hw_devices] 0]
current_hw_device $device

# 获取 JTAG-to-AXI Master
set jtag_axi [get_hw_axis]

# 复位 JTAG-to-AXI
reset_hw_axi $jtag_axi

# 读取二进制文件
set fp [open $image_file rb]
set file_data [read $fp]
close $fp

# 获取文件大小
set file_size [file size $image_file]
puts "Image size: $file_size bytes"

# 按块写入 (每次 4KB)
set block_size 4096
set num_blocks [expr {($file_size + $block_size - 1) / $block_size}]

puts "Loading image to 0x[format %08X $base_addr]..."
puts "Total blocks: $num_blocks"

for {set i 0} {$i < $num_blocks} {incr i} {
    set offset [expr {$i * $block_size}]
    set addr [expr {$base_addr + $offset}]
    set remaining [expr {$file_size - $offset}]
    set chunk_size [expr {min($block_size, $remaining)}]
    
    # 提取数据块
    set chunk [string range $file_data $offset [expr {$offset + $chunk_size - 1}]]
    
    # 转换为十六进制
    binary scan $chunk H* hex_data
    
    # 写入 AXI
    create_hw_axi_txn write_txn $jtag_axi -type write \
        -address [format 0x%08X $addr] -len [expr {$chunk_size / 4}] \
        -data $hex_data
    run_hw_axi write_txn
    delete_hw_axi_txn write_txn
    
    # 进度显示
    if {[expr {$i % 100}] == 0} {
        puts "Progress: $i / $num_blocks blocks"
    }
}

puts "Image loaded successfully!"
puts "Release reset to start Linux..."
```

执行脚本:
```bash
vivado -mode tcl -source load_image.tcl
```

### 方法 2: 使用 MCS 文件预加载 (适合 Flash 启动) (推荐)
#### Dual flash
```tcl
# 创建 Dual QSPI 配置
# 查找支持的双 Flash 配置
get_cfgmem_parts *mt25qu01g*
# 创建 dual stacked 配置
create_hw_cfgmem -hw_device [lindex [get_hw_devices xcvu13p_0] 0] \
    -mem_dev [lindex [get_cfgmem_parts {mt25qu01g-qspi-x2-dual_stacked}] 0]

```
#### 烧录
```tcl
# 在 Hardware Manager 中烧录
set cfgmem [get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xcvu13p_0] 0]]
set_property PROGRAM.ADDRESS_RANGE {use_file} $cfgmem
set_property PROGRAM.FILES [list "/home/april/Desktop/xilinx/riscv-vu13p/riscv-vu13p.runs/flash_1/flash.mcs"] $cfgmem
set_property PROGRAM.PRM_FILES {} $cfgmem
set_property PROGRAM.PRM_FILE {} $cfgmem
set_property PROGRAM.BLANK_CHECK 0 $cfgmem
set_property PROGRAM.ERASE 1 $cfgmem
set_property PROGRAM.CFG_PROGRAM 1 $cfgmem
set_property PROGRAM.VERIFY 0 $cfgmem
set_property PROGRAM.CHECKSUM 0 $cfgmem

program_hw_cfgmem $cfgmem

```

---

## 第六步: 启动 Linux

### 启动流程

1. **确认 FPGA 已编程**: bitstream 已加载
2. **确认镜像已加载**: 通过上述方法将镜像加载到 RAM
3. **连接串口**:
   ```bash
   # Linux
   minicom -D /dev/ttyUSB0 -b 115200
   # 或
   screen /dev/ttyUSB0 115200
   
   # Windows
   # 使用 PuTTY 或 TeraTerm，选择对应 COM 口，115200 波特率
   ```
4. **释放复位**: 将 reset 信号置低 (reset = 0)

### 预期输出

```
OpenSBI v1.x
   ____                    _____ ____ _____
  / __ \                  / ____|  _ \_   _|
 | |  | |_ __   ___ _ __ | (___ | |_) || |
 | |  | | '_ \ / _ \ '_ \ \___ \|  _ < | |
 | |__| | |_) |  __/ | | |____) | |_) || |_
  \____/| .__/ \___|_| |_|_____/|____/_____|
        | |
        |_|

Platform Name             : Rocket Chip FPGA
Platform Features         : medeleg
...

[    0.000000] Linux version 6.x.x ...
[    0.000000] Machine model: Rocket Chip FPGA
[    0.000000] earlycon: sifive0 at MMIO 0x0000000010000000 ...
[    0.000000] Zone ranges:
[    0.000000]   DMA32    [mem 0x0000000080000000-0x00000000bfffffff]
[    0.000000]   Normal   empty
...
[    1.000000] sifive-serial 10000000.serial: ttySIF0 at MMIO 0x10000000 ...
...

Welcome to Rocket Chip Linux!
UART: 0x10000000, 115200 baud

/ #
```

---

## 故障排除

### 问题 1: 无串口输出

- 检查 UART 引脚连接和波特率设置
- 确认 TX/RX 没有接反
- 检查 MMCM locked 信号是否有效

### 问题 2: 内核启动后卡住

- 减小 initramfs 大小
- 检查内存大小配置是否正确
- 添加 `earlyprintk` 内核参数

### 问题 3: MMCM 锁定失败

- 检查输入时钟频率是否为 200 MHz
- 检查 MMCM 参数配置

### 问题 4: 镜像加载失败

- 确认 JTAG-to-AXI 连接正确
- 检查地址范围是否在 RAM 范围内 (0x80000000 - 0xBFFFFFFF)

---

## 快速参考

### 关键参数汇总

| 参数 | 值 |
|------|-----|
| CPU | Big Rocket Core, RV64IMAFDC |
| 输入时钟 | 200 MHz (差分 LVDS) |
| 系统时钟 | 50 MHz |
| Timebase | 1 MHz |
| RAM 基地址 | 0x80000000 |
| RAM 大小 | 1 GB |
| UART 地址 | 0x10000000 |
| UART 波特率 | 115200 |
| UART 中断 | PLIC #1 |
| CLINT 地址 | 0x02000000 |
| PLIC 地址 | 0x0C000000 |
