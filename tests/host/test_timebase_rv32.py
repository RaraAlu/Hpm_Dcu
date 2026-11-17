import importlib.util
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GCC = ROOT / ".tools/hpm-gcc-13.2.0/bin/riscv32-unknown-elf-gcc"


@unittest.skipUnless(importlib.util.find_spec("unicorn"), "Unicorn 2.1.4 is required for RV32 timebase execution")
class TimebaseRv32Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import unicorn
        if unicorn.__version__ != "2.1.4":
            raise RuntimeError("Use the reviewed Unicorn 2.1.4 for this peripheral-stubbed test")
        if not GCC.exists():
            raise unittest.SkipTest("The locked HPM GCC toolchain is required")
        work = tempfile.TemporaryDirectory(prefix="dcu-timebase-rv32-")
        cls.addClassCleanup(work.cleanup)
        cls.elf = Path(work.name) / "timebase.elf"
        command = [str(GCC), "-std=c11", "-O2", "-g", "-march=rv32imac_zicsr_zifencei", "-mabi=ilp32",
                   "-msmall-data-limit=0", "-mno-relax", "-ffunction-sections", "-fdata-sections",
                   "-nostartfiles", "-Wl,--gc-sections,--no-relax", "-T", str(ROOT / "tests/host/cycle/timebase.ld")]
        for include in ("tests/host/cycle/stubs", "firmware/bsp/dcu_hpm5e31", "firmware/cfg", "firmware/ports"):
            command.extend(("-I", str(ROOT / include)))
        result = subprocess.run([*command, str(ROOT / "tests/host/cycle/cycle_timers_test.c"),
                                 "-lgcc", "-o", str(cls.elf)], capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)
        result = subprocess.run([str(GCC.with_name("riscv32-unknown-elf-nm")), "-n", str(cls.elf)],
                                check=True, capture_output=True, text=True, timeout=10)
        cls.symbols = {words[2]: int(words[0], 16) for line in result.stdout.splitlines()
                       if len(words := line.split()) == 3}

    def execute(self, origin, now, tear_at=None):
        from unicorn import Uc, UC_ARCH_RISCV, UC_MODE_RISCV32, UC_HOOK_MEM_READ
        from unicorn import riscv_const as r
        cpu = Uc(UC_ARCH_RISCV, UC_MODE_RISCV32)
        cpu.mem_map(0x80000000, 0x10000)
        cpu.mem_map(0x01200000, 0x10000)
        data = self.elf.read_bytes()
        phoff = struct.unpack_from("<I", data, 28)[0]
        phsize, phnum = struct.unpack_from("<HH", data, 42)
        for n in range(phnum):
            kind, offset, virtual, _, size, _, _, _ = struct.unpack_from("<IIIIIIII", data, phoff + n * phsize)
            if kind == 1 and size:
                cpu.mem_write(virtual, data[offset:offset + size])
        timer = self.symbols["test_machine_timer"]
        cpu.mem_write(self.symbols["machine_time_origin"], struct.pack("<Q", origin))
        cpu.mem_write(timer, struct.pack("<Q", now))
        reads = []

        def read_counter(cpu, access, address, size, value, context):
            reads.append(address - timer)
            self.assertEqual(size, 4)
            if len(reads) == tear_at:
                cpu.mem_write(timer, struct.pack("<Q", (1 << 32) + 24000))

        cpu.hook_add(UC_HOOK_MEM_READ, read_counter, begin=timer, end=timer + 7)
        cpu.reg_write(r.UC_RISCV_REG_SP, 0x0120F000)
        cpu.reg_write(r.UC_RISCV_REG_RA, 0x8000F000)
        cpu.emu_start(self.symbols["dcu_time_now_ms"], 0x8000F000, count=10000)
        self.assertEqual(cpu.reg_read(r.UC_RISCV_REG_PC), 0x8000F000, "Timebase did not return")
        return cpu.reg_read(r.UC_RISCV_REG_A0), reads

    def test_stable_counter_read_uses_high_low_high(self):
        result, reads = self.execute(1000, 1000 + 24000 * 123)
        self.assertEqual(result, 123)
        self.assertEqual(reads, [4, 0, 4])

    def test_low_word_rollover_between_reads_retries_instead_of_jumping(self):
        for tear_at in (2, 3):
            with self.subTest(tear_at=tear_at):
                result, reads = self.execute((1 << 32) - 48000, (1 << 32) - 1, tear_at)
                self.assertEqual(result, 3)
                self.assertEqual(reads, [4, 0, 4, 4, 0, 4])

    def test_rv32_division_keeps_full_width_before_millisecond_wrap(self):
        for milliseconds in (4294967, 4294968, (1 << 32) - 1, (1 << 32) + 9, 500 * 86400000):
            with self.subTest(milliseconds=milliseconds):
                result, _ = self.execute(12345, 12345 + milliseconds * 24000)
                self.assertEqual(result, milliseconds & 0xFFFFFFFF)


if __name__ == "__main__":
    unittest.main()
