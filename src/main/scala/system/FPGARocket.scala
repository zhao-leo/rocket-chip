// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.
// FPGA deployment top module for Rocket Chip
// Features: LUT-based RAM, 1GB address space, UART (from rocket-chip-blocks), RoCC accelerator support
// Uses Vivado clock primitives (IBUFDS, BUFG) for differential clock handling

package freechips.rocketchip.system

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config._
import org.chipsalliance.diplomacy.lazymodule.{LazyModule, LazyModuleImp, InModuleBody}

import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.diplomacy.{AddressSet, SimpleLazyModule, ValName, TransferSizes, RegionType}
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.devices.debug._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.resources._
import freechips.rocketchip.util._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._
import freechips.rocketchip.interrupts._

import sifive.blocks.devices.uart._

// ============================================================================
// Vivado Clock Primitives (BlackBox wrappers)
// Vivado will automatically recognize these as built-in primitives
// ============================================================================

/** IBUFDS: Differential Input Buffer for Vivado */
class IBUFDS extends BlackBox {
  val io = IO(new Bundle {
    val O = Output(Clock())
    val I = Input(Clock())
    val IB = Input(Clock())
  })
}

/** BUFG: Global Clock Buffer for Vivado */
class BUFG extends BlackBox {
  val io = IO(new Bundle {
    val O = Output(Clock())
    val I = Input(Clock())
  })
}

/** MMCME2_BASE: Mixed-Mode Clock Manager for Vivado
  * Used to divide 200MHz input clock to 50MHz output clock
  * Parameters are set via generic map for 200MHz -> 50MHz:
  *   CLKFBOUT_MULT_F = 5.0 (200 * 5 = 1000 MHz VCO)
  *   CLKOUT0_DIVIDE_F = 20.0 (1000 / 20 = 50 MHz)
  *   CLKIN1_PERIOD = 5.0 (200 MHz = 5ns period)
  */
class MMCME2_BASE extends BlackBox(Map(
  "CLKFBOUT_MULT_F" -> 5.0,      // VCO = 200 * 5 = 1000 MHz
  "CLKOUT0_DIVIDE_F" -> 20.0,    // CLKOUT0 = 1000 / 20 = 50 MHz
  "CLKIN1_PERIOD" -> 5.0,        // 200 MHz input = 5ns period
  "DIVCLK_DIVIDE" -> 1
)) {
  val io = IO(new Bundle {
    val CLKOUT0 = Output(Clock())
    val CLKOUT0B = Output(Clock())
    val CLKOUT1 = Output(Clock())
    val CLKOUT1B = Output(Clock())
    val CLKOUT2 = Output(Clock())
    val CLKOUT2B = Output(Clock())
    val CLKOUT3 = Output(Clock())
    val CLKOUT3B = Output(Clock())
    val CLKOUT4 = Output(Clock())
    val CLKOUT5 = Output(Clock())
    val CLKOUT6 = Output(Clock())
    val CLKFBOUT = Output(Clock())
    val CLKFBOUTB = Output(Clock())
    val LOCKED = Output(Bool())
    val CLKIN1 = Input(Clock())
    val PWRDWN = Input(Bool())
    val RST = Input(Bool())
    val CLKFBIN = Input(Clock())
  })
}

/** Clock generator using Vivado primitives
  * Input: 200MHz differential clock (clock_p, clock_n)
  * Output: 50MHz system clock (clk_out) and locked signal
  */
class VivadoClockGen extends RawModule {
  val io = IO(new Bundle {
    val clk_p = Input(Clock())
    val clk_n = Input(Clock())
    val clk_out = Output(Clock())
    val locked = Output(Bool())
  })

  // Instantiate IBUFDS for differential clock input (200 MHz)
  val ibufds = Module(new IBUFDS)
  ibufds.io.I := io.clk_p
  ibufds.io.IB := io.clk_n

  // Instantiate MMCME2_BASE for clock division (200 MHz -> 50 MHz)
  val mmcm = Module(new MMCME2_BASE)
  mmcm.io.CLKIN1 := ibufds.io.O
  mmcm.io.RST := false.B
  mmcm.io.PWRDWN := false.B

  // Feedback path for MMCM
  val bufg_fb = Module(new BUFG)
  bufg_fb.io.I := mmcm.io.CLKFBOUT
  mmcm.io.CLKFBIN := bufg_fb.io.O

  // Output clock through BUFG (50 MHz)
  val bufg_out = Module(new BUFG)
  bufg_out.io.I := mmcm.io.CLKOUT0

  io.clk_out := bufg_out.io.O
  io.locked := mmcm.io.LOCKED
}

// ============================================================================
// FPGA Memory Configuration - 1GB Address Space
// ============================================================================

/** Configuration for 1GB memory address space */
class WithFPGA1GBMemPort extends Config((site, here, up) => {
  case ExtMem => Some(MemoryPortParams(MasterPortParams(
    base = BigInt("80000000", 16),     // 0x8000_0000
    size = BigInt("40000000", 16),     // 1GB = 0x4000_0000
    beatBytes = site(MemoryBusKey).beatBytes,
    idBits = 4), 1))
})

/** No external memory port - use on-chip RAM only */
class WithNoExtMemPort extends Config((site, here, up) => {
  case ExtMem => None
})

// ============================================================================
// UART Configuration (using rocket-chip-blocks)
// ============================================================================

/** Add UART peripheral at specified address using rocket-chip-blocks implementation */
class WithFPGAUART(address: BigInt = 0x10000000L, baudRate: BigInt = 115200) extends Config((site, here, up) => {
  case PeripheryUARTKey => Seq(UARTParams(
    address = address,
    initBaudRate = baudRate
  ))
})

// ============================================================================
// LUT-based AXI4 RAM for FPGA (uses distributed RAM instead of BRAM)
// ============================================================================

class FPGAAXI4RAM(
    address: AddressSet,
    cacheable: Boolean = true,
    executable: Boolean = true,
    beatBytes: Int = 4,
    devName: Option[String] = None,
    errors: Seq[AddressSet] = Nil)
  (implicit p: Parameters) extends SimpleLazyModule {

  val node = AXI4SlaveNode(Seq(AXI4SlavePortParameters(
    slaves = Seq(AXI4SlaveParameters(
      address = Seq(address),
      resources = devName.map(n => new SimpleDevice(n, Seq("sifive,sram0")).reg("mem")).toSeq.flatten,
      regionType = if (cacheable) RegionType.UNCACHED else RegionType.IDEMPOTENT,
      executable = executable,
      supportsRead = TransferSizes(1, beatBytes),
      supportsWrite = TransferSizes(1, beatBytes),
      interleavedId = Some(0))),
    beatBytes = beatBytes)))

  private val outer = this

  override lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val (in, edge) = outer.node.in(0)

    val laneDataBits = 8
    val lanes = beatBytes
    val memSizeBits = log2Ceil(address.mask.toInt + 1)

    // Limit on-chip memory to reasonable size for LUT-based RAM
    val maxOnChipSizeBits = 16 // 64KB max
    val actualSizeBits = memSizeBits.min(maxOnChipSizeBits)
    val actualSize = 1 << actualSizeBits

    // Create LUT-based memory using SyncReadMem (distributed RAM)
    val mem = SyncReadMem(actualSize / lanes, Vec(lanes, UInt(laneDataBits.W)))
    val address_base = address.base

    def extractAddr(addr: UInt): UInt = {
      val addrBits = addr - address_base.U
      addrBits(actualSizeBits - 1, log2Ceil(lanes))
    }

    // Write channel state
    val w_full = RegInit(false.B)
    val w_id = Reg(UInt(in.params.idBits.W))
    val w_echo = Reg(BundleMap(in.params.echoFields))

    val wdata = VecInit(Seq.tabulate(lanes) { i =>
      in.w.bits.data((i + 1) * laneDataBits - 1, i * laneDataBits)
    })

    when (in.aw.fire && in.w.fire) {
      val addr = extractAddr(in.aw.bits.addr)
      mem.write(addr, wdata, in.w.bits.strb.asBools)
      w_full := true.B
      w_id := in.aw.bits.id
      w_echo :<= in.aw.bits.echo
    }

    when (in.b.fire) { w_full := false.B }

    in.aw.ready := !w_full
    in.w.ready := !w_full && in.aw.valid
    in.b.valid := w_full
    in.b.bits.id := w_id
    in.b.bits.resp := AXI4Parameters.RESP_OKAY
    in.b.bits.echo :<= w_echo

    // Read channel state
    val r_full = RegInit(false.B)
    val r_id = Reg(UInt(in.params.idBits.W))
    val r_echo = Reg(BundleMap(in.params.echoFields))

    when (in.ar.fire) {
      r_id := in.ar.bits.id
      r_echo :<= in.ar.bits.echo
      r_full := true.B
    }

    val ren = in.ar.fire
    val rdata = mem.read(extractAddr(in.ar.bits.addr), ren)

    when (in.r.fire) { r_full := false.B }

    in.ar.ready := !r_full
    in.r.valid := r_full
    in.r.bits.id := r_id
    in.r.bits.data := Cat(rdata.reverse)
    in.r.bits.resp := AXI4Parameters.RESP_OKAY
    in.r.bits.last := true.B
    in.r.bits.echo :<= r_echo
  }
}

// ============================================================================
// FPGA Memory Wrapper - connects to AXI4 mem port with LUT-based RAM
// ============================================================================

class FPGASimAXIMem(edge: AXI4EdgeParameters, size: BigInt, base: BigInt = 0)(implicit p: Parameters) extends SimpleLazyModule {
  val node = AXI4MasterNode(List(edge.master))

  // On-chip memory size - keep small for LUT-based implementation
  val onChipMemSize: BigInt = BigInt(64 * 1024) // 64KB

  val srams = AddressSet.misaligned(base, onChipMemSize).map { aSet =>
    LazyModule(new FPGAAXI4RAM(
      address = aSet,
      beatBytes = edge.bundle.dataBits / 8))
  }

  val xbar = AXI4Xbar()
  srams.foreach { s => s.node := AXI4Buffer() := AXI4Fragmenter() := xbar }
  xbar := node
  val io_axi4 = InModuleBody { node.makeIOs() }
}

object FPGASimAXIMem {
  def connectMem(dut: CanHaveMasterAXI4MemPort)(implicit p: Parameters): Seq[FPGASimAXIMem] = {
    dut.mem_axi4.zip(dut.memAXI4Node.in).map { case (io, (_, edge)) =>
      val mem = LazyModule(new FPGASimAXIMem(edge, base = p(ExtMem).get.master.base, size = p(ExtMem).get.master.size))
      Module(mem.module).suggestName("fpga_mem")
      mem.io_axi4.head <> io
      mem
    }.toSeq
  }

  def connectMMIO(dut: CanHaveMasterAXI4MMIOPort)(implicit p: Parameters): Seq[FPGASimAXIMem] = {
    dut.mmio_axi4.zip(dut.mmioAXI4Node.in).map { case (io, (_, edge)) =>
      val mmio_mem = LazyModule(new FPGASimAXIMem(edge, base = p(ExtBus).get.base, size = 4096))
      Module(mmio_mem.module).suggestName("fpga_mmio_mem")
      mmio_mem.io_axi4.head <> io
      mmio_mem
    }.toSeq
  }
}

// ============================================================================
// FPGA Rocket System with UART (using rocket-chip-blocks)
// ============================================================================

class FPGARocketSystem(implicit p: Parameters) extends RocketSubsystem
    with HasAsyncExtInterrupts
    with CanHaveMasterAXI4MemPort
    with CanHaveMasterAXI4MMIOPort
    with CanHaveSlaveAXI4Port
    with HasPeripheryUART {
  val bootROM = p(BootROMLocated(location)).map { BootROM.attach(_, this, CBUS) }
  val maskROMs = p(MaskROMLocated(location)).map { MaskROM.attach(_, this, CBUS) }

  override lazy val module = new FPGARocketSystemModuleImp(this)
}

class FPGARocketSystemModuleImp[+L <: FPGARocketSystem](_outer: L) extends RocketSubsystemModuleImp(_outer)
    with HasRTCModuleImp
    with HasExtInterruptsModuleImp
    with DontTouch

// ============================================================================
// FPGA System Wrapper with on-chip memory (no debug, UART only)
// ============================================================================

class FPGARocketSystemWrapper(implicit p: Parameters) extends LazyModule {
  val rocketSystem = LazyModule(new FPGARocketSystem)

  override lazy val module = new FPGARocketSystemWrapperImp(this)
}

class FPGARocketSystemWrapperImp(outer: FPGARocketSystemWrapper)(implicit p: Parameters) extends LazyModuleImp(outer) {
  val ldut = outer.rocketSystem
  val dut = ldut.module

  val io = IO(new Bundle {
    val sys_clock = Input(Clock())
    val sys_reset = Input(Bool())
    // UART interface
    val uart = new UARTPortIO(p(PeripheryUARTKey).head)
  })

  // Clock domain setup
  ldut.io_clocks.get.elements.values.foreach(_.clock := io.sys_clock)

  // Reset handling - use synchronous reset to match SubsystemResetSchemeKey default
  val dut_reset = io.sys_reset.asBool
  ldut.io_clocks.get.elements.values.foreach(_.reset := dut_reset)

  dut.dontTouchPorts()
  dut.tieOffInterrupts()

  // Memory Connection - Use LUT-based RAM
  FPGASimAXIMem.connectMem(ldut)
  FPGASimAXIMem.connectMMIO(ldut)

  // Tie off slave port if present
  ldut.l2_frontend_bus_axi4.foreach { a =>
    a.ar.valid := false.B
    a.ar.bits := DontCare
    a.aw.valid := false.B
    a.aw.bits := DontCare
    a.w.valid := false.B
    a.w.bits := DontCare
    a.r.ready := false.B
    a.b.ready := false.B
  }

  // Tie off debug interface (no SimDTM, no JTAG)
  ldut.debug.foreach { debug =>
    debug.clock := false.B.asClock
    debug.reset := true.B.asAsyncReset
    debug.dmactiveAck := false.B
    debug.disableDebug.foreach(_ := true.B)

    debug.systemjtag.foreach { sj =>
      sj.jtag.TCK := false.B.asClock
      sj.jtag.TMS := true.B
      sj.jtag.TDI := true.B
      sj.jtag.TRSTn.foreach(_ := true.B)
      sj.reset := true.B.asAsyncReset
      sj.mfr_id := 0.U
      sj.part_number := 0.U
      sj.version := 0.U
    }

    debug.clockeddmi.foreach { dmi =>
      dmi.dmi.req.valid := false.B
      dmi.dmi.req.bits := DontCare
      dmi.dmi.resp.ready := true.B
      dmi.dmiClock := false.B.asClock
      dmi.dmiReset := true.B.asAsyncReset
    }

    debug.apb.foreach { apb =>
      apb.psel := false.B
      apb.penable := false.B
      apb.pwrite := false.B
      apb.paddr := 0.U
      apb.pwdata := 0.U
      apb.pauser := DontCare
      apb.clock := false.B.asClock
      apb.reset := true.B.asAsyncReset
    }
  }

  // Tie off reset control
  ldut.resetctrl.foreach { rcio =>
    rcio.hartIsInReset.foreach(_ := io.sys_reset)
  }

  // Tie off PSD
  ldut.psd.psd.foreach(_ <> 0.U.asTypeOf(new PSDTestMode()))

  // UART connection (using rocket-chip-blocks UART)
  ldut.uart.headOption.foreach { uartPort =>
    io.uart <> uartPort
  }
}

// ============================================================================
// FPGA Top Module: FPGARocketTop (RawModule with differential clock)
// Uses Vivado clock primitives (IBUFDS, MMCME2_BASE, BUFG)
// Input: 200MHz differential clock -> Output: 50MHz system clock
// ============================================================================

/**
  * Top-level FPGA module with differential clock, reset, and UART.
  * Ports:
  *   - clock_p, clock_n: 200MHz Differential clock inputs (uses IBUFDS + MMCM + BUFG)
  *   - reset: Active-high reset
  *   - uart_tx, uart_rx: UART interface
  *
  * Clock: 200MHz input -> 50MHz system clock (via MMCME2_BASE)
  */
class FPGARocketTop(implicit p: Parameters) extends RawModule {
  // Differential clock inputs (200 MHz)
  val clock_p = IO(Input(Clock()))
  val clock_n = IO(Input(Clock()))

  // Reset (active-high)
  val reset = IO(Input(Bool()))

  // UART interface
  val uart_tx = IO(Output(Bool()))
  val uart_rx = IO(Input(Bool()))

  // Use Vivado clock primitives for differential clock with MMCM
  // 200 MHz -> 50 MHz
  val clkGen = Module(new VivadoClockGen)
  clkGen.io.clk_p := clock_p
  clkGen.io.clk_n := clock_n
  val sys_clock = clkGen.io.clk_out
  val mmcm_locked = clkGen.io.locked

  // Reset synchronizer (2-stage synchronizer for metastability)
  // Also hold reset until MMCM is locked
  val reset_sync = withClockAndReset(sys_clock, reset.asAsyncReset) {
    val r0 = RegNext(reset || !mmcm_locked, true.B)
    val r1 = RegNext(r0, true.B)
    r1
  }

  // Instantiate the system wrapper
  withClockAndReset(sys_clock, reset_sync.asAsyncReset) {
    val wrapper = Module(LazyModule(new FPGARocketSystemWrapper).module)

    wrapper.io.sys_clock := sys_clock
    wrapper.io.sys_reset := reset_sync

    // Connect UART ports
    uart_tx := wrapper.io.uart.txd
    wrapper.io.uart.rxd := uart_rx

    // Handle optional CTS/RTS signals
    wrapper.io.uart.cts_n.foreach(_ := false.B)
  }
}

// ============================================================================
// FPGA Configurations
// ============================================================================

/** Config fragment to disable debug module */
class WithNoDebugModule extends Config((site, here, up) => {
  case DebugModuleKey => None
})

/** Config fragment to use asynchronous reset scheme */
class WithAsyncReset extends Config((site, here, up) => {
  case SubsystemResetSchemeKey => ResetAsynchronous
})

/** Base configuration for FPGA deployment (no debug) */
class FPGABaseConfig extends Config(
  new WithFPGAUART() ++                            // Add UART peripheral (rocket-chip-blocks)
  new WithFPGA1GBMemPort ++                        // 1GB address space
  new WithDefaultMMIOPort ++                       // MMIO port
  new WithNoSlavePort ++                           // No slave port needed
  new WithNoDebugModule ++                         // No debug module (UART only)
  new WithTimebase(BigInt(1000000)) ++             // 1 MHz timebase
  new WithDTS("freechips,rocketchip-fpga", Nil) ++
  new WithNExtTopInterrupts(2) ++
  new BaseSubsystemConfig
)

/** Default FPGA configuration with Big Rocket core and RoCC accelerator */
class DefaultFPGARocketConfig extends Config(
  new WithRoccExample ++                           // Add RoCC accelerator examples
  new freechips.rocketchip.rocket.WithNSmallCores(1) ++
  new WithCoherentBusTopology ++
  new FPGABaseConfig
)
