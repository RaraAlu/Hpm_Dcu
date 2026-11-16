import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class FreeRtosVendorTest(unittest.TestCase):
    def test_pinned_vendor_inventory_and_bytes(self):
        lock = json.loads((ROOT / "tools/cfg/freertos.lock.json").read_text())
        vendor = ROOT / lock["local_path"]
        files = sorted(file for file in vendor.rglob("*") if file.is_file())
        self.assertEqual(len(files), lock["file_count"])
        manifest = "".join(
            file.relative_to(vendor).as_posix() + "\0" +
            hashlib.sha256(file.read_bytes()).hexdigest() + "\n"
            for file in files
        )
        self.assertEqual(hashlib.sha256(manifest.encode()).hexdigest(), lock["manifest_sha256"])
        version = (vendor / "Source/include/task.h").read_text()
        self.assertIn(f'"{lock["kernel_version"]}"', version)
        for upstream, local in lock["renamed_documents"].items():
            self.assertFalse((vendor / upstream).exists())
            self.assertTrue((vendor / local).is_file())

    def test_static_allocation_and_high_priority_irq_contract(self):
        config = (ROOT / "firmware/cfg/FreeRTOSConfig.h").read_text()
        self.assertIn("#define configSUPPORT_STATIC_ALLOCATION 1", config)
        self.assertIn("#define configSUPPORT_DYNAMIC_ALLOCATION 0", config)
        self.assertIn("#define configMAX_SYSCALL_INTERRUPT_PRIORITY DCU_IRQ_KERNEL_PRIORITY", config)
        self.assertIn("#define configUSE_TICKLESS_IDLE 0", config)
        cmake = (ROOT / "CMakeLists.txt").read_text()
        self.assertIn("set(CONFIG_FREERTOS_HEAP custom)", cmake)
        self.assertIn("-DUSE_SYSCALL_INTERRUPT_PRIORITY=1", cmake)

    def test_local_patch_bytes_match_the_declared_provenance(self):
        lock = json.loads((ROOT / "tools/cfg/freertos.lock.json").read_text())
        patches = lock["content_changes"]
        self.assertEqual(len(patches), 2)
        for patch in patches:
            data = (ROOT / lock["local_path"] / patch["path"]).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), patch["patched_sha256"])
            self.assertNotEqual(patch["upstream_sha256"], patch["patched_sha256"])
            self.assertTrue(patch["reason"])


if __name__ == "__main__":
    unittest.main()
