import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from image_guard import FLASH_BASE, RETAINED_START, inspect_image


def make_elf(segments=None, machine=243, entry=FLASH_BASE + 0x3000):
    if segments is None:
        segments = [(FLASH_BASE + 0x400, 4), (FLASH_BASE + 0x3000, 16)]
    ident = b"\x7fELF\x01\x01\x01" + bytes(9)
    offset = 52 + 32 * len(segments)
    headers, payload = [], bytearray()
    for address, size in segments:
        # Different virtual and physical addresses exercise the Flash LMA check.
        headers.append(struct.pack("<IIIIIIII", 1, offset, 0, address, size, max(size, 16), 5, 4))
        payload.extend(bytes(size))
        offset += size
    header = struct.pack("<16sHHIIIIIHHHHHH", ident, 2, machine, 1, entry, 52, 0,
                         0, 52, 32, len(segments), 0, 0, 0)
    return header + b"".join(headers) + payload


class ImageGuardTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.elf = Path(self.directory.name) / "image.elf"

    def inspect(self, data=None, **options):
        self.elf.write_bytes(make_elf() if data is None else data)
        return inspect_image(self.elf, **options)

    def test_valid_image_uses_physical_load_addresses(self):
        report = self.inspect()
        self.assertEqual(report["erase_start"], FLASH_BASE)
        self.assertEqual(report["erase_end_exclusive"], FLASH_BASE + 0x4000)
        self.assertFalse(report["hardware_geometry_verified"])

    def test_retained_region_direct_overlap_is_rejected(self):
        data = make_elf([(FLASH_BASE + 0x400, 4), (RETAINED_START, 4)])
        with self.assertRaisesRegex(ValueError, "retained"):
            self.inspect(data)

    def test_hole_spanning_retained_region_is_rejected(self):
        data = make_elf([(FLASH_BASE + 0x400, 4), (RETAINED_START + 0x10000, 4)])
        with self.assertRaisesRegex(ValueError, "retained"):
            self.inspect(data)

    def test_erase_rounding_can_make_an_otherwise_small_image_unsafe(self):
        with self.assertRaisesRegex(ValueError, "retained"):
            self.inspect(sector_size=0x100000)

    def test_image_may_end_exactly_before_retained_region(self):
        data = make_elf([(FLASH_BASE + 0x400, 4), (RETAINED_START - 4, 4)])
        self.assertEqual(self.inspect(data)["erase_end_exclusive"], RETAINED_START)

    def test_unsupported_cpu_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "RISC-V"):
            self.inspect(make_elf(machine=62))

    def test_wrong_entry_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "boot layout"):
            self.inspect(make_elf(entry=FLASH_BASE))

    def test_wrong_first_load_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "boot layout"):
            self.inspect(make_elf([(FLASH_BASE, 4)]))

    def test_nonempty_ram_load_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            self.inspect(make_elf([(0x200000, 16)]))

    def test_out_of_capacity_load_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            self.inspect(make_elf([(FLASH_BASE + 0x400, 4), (FLASH_BASE + 0x100000, 4)]))

    def test_zero_sized_noload_ram_does_not_expand_flash_erase(self):
        data = make_elf([(FLASH_BASE + 0x400, 4), (FLASH_BASE + 0x3000, 16), (0x200000, 0)])
        self.assertEqual(len(self.inspect(data)["loads"]), 2)

    def test_truncated_headers_are_rejected(self):
        for data in (b"", make_elf()[:30], make_elf()[:70]):
            with self.subTest(length=len(data)), self.assertRaises(ValueError):
                self.inspect(data)

    def test_truncated_load_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Truncated"):
            self.inspect(make_elf()[:-1])

    def test_big_endian_image_is_rejected(self):
        data = bytearray(make_elf())
        data[5] = 2
        with self.assertRaisesRegex(ValueError, "little-endian"):
            self.inspect(data)

    def test_invalid_sector_size_is_rejected(self):
        for size in (0, -1, 3000):
            with self.subTest(size=size), self.assertRaises(ValueError):
                self.inspect(sector_size=size)


if __name__ == "__main__":
    unittest.main()
