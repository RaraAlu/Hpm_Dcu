import importlib.util
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SDK = ROOT / "third_party/hpm_sdk"
PORT = SDK / "middleware/FreeRTOS/Source/portable/GCC/RISC-V"
GCC = ROOT / ".tools/hpm-gcc-13.2.0/bin/riscv32-unknown-elf-gcc"


class FreeRtosPortContractTest(unittest.TestCase):
    def test_priority_masks_are_part_of_saved_and_initial_contexts(self):
        extension = (PORT / "chip_specific_extensions/HPMicro/freertos_risc_v_chip_specific_extensions.h").read_text()
        assembly = (PORT / "portASM.S").read_text()
        self.assertIn("portasmADDITIONAL_CONTEXT_IRQ_SIZE", extension)
        self.assertIn("csrr t0, mie", extension)
        self.assertIn("csrw mie, t0", extension)
        self.assertIn("portasmINITIALISE_ADDITIONAL_REGISTERS", assembly)


@unittest.skipUnless(importlib.util.find_spec("unicorn"), "Unicorn 2.1.4 is required for RV32 kernel execution")
class FreeRtosKernelExecutionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not GCC.exists():
            raise unittest.SkipTest("The locked HPM GCC toolchain is required")
        import unicorn
        if unicorn.__version__ != "2.1.4":
            raise RuntimeError("Use the reviewed Unicorn 2.1.4 for this peripheral-stubbed test")
        directory = tempfile.TemporaryDirectory(prefix="dcu-kernel-execution-")
        cls.addClassCleanup(directory.cleanup)
        cls.elf = Path(directory.name) / "kernel.elf"
        flags = ["-std=c11", "-O2", "-g", "-march=rv32imac_zicsr_zifencei", "-mabi=ilp32",
                 "-msmall-data-limit=0", "-mno-relax", "-ffunction-sections", "-fdata-sections",
                 "-DCONFIG_FREERTOS=1", "-DUSE_SYSCALL_INTERRUPT_PRIORITY=1", "-DportasmHAS_MTIME=1"]
        for include in (ROOT / "tests/host/freertos/stubs", ROOT / "firmware/cfg", PORT,
                        PORT / "chip_specific_extensions/HPMicro", PORT.parents[2] / "include",
                        SDK / "arch", SDK / "arch/riscv/intc", SDK / "drivers/inc",
                        SDK / "soc/HPM5E00/HPM5E31", SDK / "soc/HPM5E00/ip"):
            flags.extend(("-I", str(include)))
        command = [str(GCC), *flags, "-nostartfiles", "-Wl,--gc-sections,--no-relax",
                   "-T", str(ROOT / "tests/host/freertos/fixture.ld"),
                   str(ROOT / "tests/host/freertos/kernel_fixture.c"),
                   str(PORT.parents[2] / "tasks.c"), str(PORT.parents[2] / "list.c"),
                   str(PORT / "port.c"), str(PORT / "portASM.S"), "-lc", "-lgcc", "-o", str(cls.elf)]
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)
        result = subprocess.run([str(GCC.with_name("riscv32-unknown-elf-nm")), "-n", str(cls.elf)],
                                check=True, capture_output=True, text=True, timeout=10)
        cls.symbols = {words[2]: int(words[0], 16) for line in result.stdout.splitlines()
                       if len(words := line.split()) == 3}

    def setUp(self):
        from unicorn import Uc, UC_ARCH_RISCV, UC_MODE_RISCV32, UC_HOOK_CODE, UC_HOOK_INTR
        from unicorn import riscv_const as r
        self.r = r
        self.cpu = Uc(UC_ARCH_RISCV, UC_MODE_RISCV32)
        for base, size in ((0x80000000, 0x80000), (0x01200000, 0x20000),
                           (0xE4000000, 0x202000), (0xE6000000, 0x10000)):
            self.cpu.mem_map(base, size)
        data = self.elf.read_bytes()
        phoff = struct.unpack_from("<I", data, 28)[0]
        phsize, phnum = struct.unpack_from("<HH", data, 42)
        for n in range(phnum):
            kind, offset, virtual, _, size, _, _, _ = struct.unpack_from("<IIIIIIII", data, phoff + n * phsize)
            if kind == 1 and size:
                self.cpu.mem_write(virtual, data[offset:offset + size])
        self.cpu.reg_write(r.UC_RISCV_REG_SP, 0x0121C000)
        self.cpu.reg_write(r.UC_RISCV_REG_MSTATUS, 0x1800)
        self.cpu.reg_write(r.UC_RISCV_REG_MIE, 0x800)
        self.cpu.hook_add(UC_HOOK_INTR, self.trap)
        self.cpu.hook_add(UC_HOOK_CODE, self.breakpoint)
        self.stops = set()
        self.hit = None
        self.hpm_csrs = {}

    def word(self, symbol):
        return int.from_bytes(self.cpu.mem_read(self.symbols[symbol], 4), "little")

    def breakpoint(self, cpu, address, size, _):
        if address in self.stops:
            self.hit = address
            cpu.emu_stop()

    def trap(self, cpu, number, _):
        r = self.r
        pc = cpu.reg_read(r.UC_RISCV_REG_PC)
        instruction = int.from_bytes(cpu.mem_read(pc, 4), "little")
        previous = int.from_bytes(cpu.mem_read(pc - 4, 4), "little")
        if previous == 0x73 or (previous & 0x7F == 0x73 and (previous >> 20) in (0x7CB, 0x7CD)):
            pc, instruction = pc - 4, previous
        # HPM MCCTL registers are peripheral stand-ins, not implemented by Unicorn.
        if instruction & 0x7F == 0x73 and (instruction >> 20) in (0x7CB, 0x7CD):
            rd = (instruction >> 7) & 31
            rs1 = (instruction >> 15) & 31
            csr = instruction >> 20
            old = self.hpm_csrs.get(csr, 0)
            value = cpu.reg_read(r.UC_RISCV_REG_X0 + rs1)
            operation = (instruction >> 12) & 7
            if operation == 1:
                self.hpm_csrs[csr] = value
            elif operation == 2 and rs1:
                self.hpm_csrs[csr] = old | value
            if rd:
                cpu.reg_write(r.UC_RISCV_REG_X0 + rd, old)
            cpu.reg_write(r.UC_RISCV_REG_PC, pc + 4)
            return
        if instruction == 0x73:  # Machine ecall: enter the production kernel trap path.
            status = cpu.reg_read(r.UC_RISCV_REG_MSTATUS)
            cpu.reg_write(r.UC_RISCV_REG_MSTATUS, (status & ~0x88) | ((status & 8) << 4) | 0x1800)
            cpu.reg_write(r.UC_RISCV_REG_MEPC, pc)
            cpu.reg_write(r.UC_RISCV_REG_MCAUSE, 11)
            cpu.reg_write(r.UC_RISCV_REG_PC, self.symbols["freertos_risc_v_trap_handler"])
            return
        raise AssertionError(f"Unexpected emulated trap {number}, pc={pc:#x}, instruction={instruction:#x}")

    def run_to(self, start, *symbols):
        self.stops = {self.symbols[name] for name in symbols}
        self.hit = None
        self.cpu.emu_start(start, 0xFFFFFFFF, count=200000)
        self.assertIn(self.hit, self.stops, "Kernel did not reach its expected task state")
        self.assertEqual(self.word("fixture_failure"), 0)
        return self.cpu.reg_read(self.r.UC_RISCV_REG_PC)

    def assert_unmasked(self):
        self.assertEqual(self.cpu.reg_read(self.r.UC_RISCV_REG_MIE) & 0x880, 0x880)
        self.assertEqual(int.from_bytes(self.cpu.mem_read(0xE4200000, 4), "little"), 0)
        self.assertEqual(self.word("xCriticalNesting"), 0)
        self.assertEqual(self.cpu.reg_read(self.r.UC_RISCV_REG_MSCRATCH), 0)
        self.assertTrue(self.cpu.reg_read(self.r.UC_RISCV_REG_MSTATUS) & 8)

    def enter_irq(self, irq):
        r = self.r
        pc = self.cpu.reg_read(r.UC_RISCV_REG_PC)
        status = self.cpu.reg_read(r.UC_RISCV_REG_MSTATUS)
        self.cpu.reg_write(r.UC_RISCV_REG_MEPC, pc)
        self.cpu.reg_write(r.UC_RISCV_REG_MSTATUS, (status & ~0x88) | ((status & 8) << 4) | 0x1800)
        self.cpu.reg_write(r.UC_RISCV_REG_MCAUSE, 0x8000000B)
        return self.symbols[f"default_isr_{irq}"]

    def test_real_kernel_blocks_and_repeatedly_wakes_via_production_contexts(self):
        pc = self.run_to(self.symbols["fixture_start"], "fixture_worker")
        self.assert_unmasked()
        pc = self.run_to(pc, "prvIdleTask")
        self.assert_unmasked()
        self.assertEqual(self.word("fixture_phase"), 1)
        pc = self.run_to(self.enter_irq(1), "prvIdleTask")
        self.assert_unmasked()
        self.assertEqual(self.word("fixture_phase"), 2)
        self.run_to(self.enter_irq(1), "fixture_done")
        self.assert_unmasked()
        self.assertEqual(self.word("fixture_phase"), 3)

    def test_nested_fast_irq_preserves_the_interrupted_kernel_mask(self):
        pc = self.run_to(self.symbols["fixture_start"], "fixture_worker")
        self.run_to(pc, "prvIdleTask")
        pc = self.run_to(self.enter_irq(1), "portExitCriticalFromIsr")
        before = self.cpu.reg_read(self.r.UC_RISCV_REG_MIE)
        threshold = bytes(self.cpu.mem_read(0xE4200000, 4))
        self.assertEqual(int.from_bytes(threshold, "little"), 3)
        self.run_to(self.enter_irq(5), "portExitCriticalFromIsr")
        self.assertEqual(self.cpu.reg_read(self.r.UC_RISCV_REG_MIE), before)
        self.assertEqual(bytes(self.cpu.mem_read(0xE4200000, 4)), threshold)
        self.assertEqual(self.word("fixture_nested_hits"), 1)
        self.run_to(pc, "prvIdleTask")
        self.assert_unmasked()


if __name__ == "__main__":
    unittest.main()
