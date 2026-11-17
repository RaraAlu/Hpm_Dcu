import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'tools/protocol'
INPUTS = SOURCE / 'cfg'
spec = importlib.util.spec_from_file_location('dcu_protocol_generator', SOURCE / 'generate.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
ESI = 'protocol/ethercat/HPM_ENCOS_4CAN_28AXIS_IMU.xml'
SII = ESI.replace('.xml', '.bin')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def xml_contract(node):
    return node.tag, sorted(node.attrib.items()), (node.text or '').strip(), [xml_contract(c) for c in node]


def categories(data):
    result, offset = {}, 128
    while True:
        kind = struct.unpack_from('<H', data, offset)[0]
        if kind == 0xFFFF:
            return result, offset
        words = struct.unpack_from('<H', data, offset + 2)[0]
        if kind in result or not words or offset + 4 + words * 2 > len(data):
            raise ValueError('Invalid SII category')
        result[kind] = data[offset + 4:offset + 4 + words * 2]
        offset += 4 + words * 2


class ProtocolGenerationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.schema = json.loads((INPUTS / '接口定义.json').read_text(encoding='utf-8'))
        cls.generated = generator.outputs()
        cls.xml = ET.fromstring(cls.generated[ESI])
        cls.objects, cls.pdos = generator.expand(cls.schema)

    def test_generation_works_without_existing_outputs_and_is_reproducible(self):
        with tempfile.TemporaryDirectory(prefix='dcu-protocol-') as directory:
            root = Path(directory)
            source = root / 'tools/protocol'
            (source / 'cfg').mkdir(parents=True)
            for name in ('generate.py', 'cfg/接口定义.json', 'cfg/设备模板.xml'):
                shutil.copy2(SOURCE / name, source / name)
            command = [sys.executable, str(source / 'generate.py')]
            for _ in range(2):
                subprocess.run([*command, '--output-root', str(root)], check=True, capture_output=True)
                for name, content in self.generated.items():
                    self.assertEqual((root / name).read_bytes(), content, name)
            subprocess.run(command, check=True, capture_output=True)
            target = root / SII
            target.write_bytes(b'changed')
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 1)
            self.assertEqual(target.read_bytes(), b'changed')

    def test_checked_in_outputs_are_current(self):
        for name, content in self.generated.items():
            self.assertEqual((ROOT / name).read_bytes(), content, name)

    def test_wire_c_tokens_match_the_pre_migration_contract(self):
        # Frozen from c8ea19f, before the new source definition/generator existed.
        expected = {
            'encos/encos_interface.h': '2b74c842d03b6c4b96322ca0cfe2d2c15d5ffe211132e876d7c4df743f0fbf16',
            'encos/encos_storage.inc': '7ee9b5d6bcb4773d86fda06475d35b47def2ecb47c639528f90ce069b8dbddfd',
            'ethercat/gatewayObjects.h': '1e8da1c060f6cb76fea7c4b3550792b3dbd5bae3988521931800c23c27171d45',
        }
        for name, checksum in expected.items():
            text = self.generated['firmware/generated/' + name].decode()
            text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
            text = re.sub(r'^#define MC_.*$', '', text, flags=re.M)
            text = re.sub(r'0x[0-9a-fA-F]+', lambda m: m[0].lower(), text)
            tokens = ' '.join(re.findall(r'\w+|[^\w\s]', text))
            self.assertEqual(digest(tokens.encode()), checksum, name)
        self.assertEqual(self.schema['constants'], {
            'MC_INTERFACE_VERSION': '0x00010002u', 'MC_AXIS_ARM': '0x0001u',
            'MC_AXIS_STOP': '0x0002u', 'MC_AXIS_RESET': '0x0004u',
            'MC_AXIS_START_MOTION': '0x0008u', 'MC_STATUS_QUERY_SOURCE': '0x1000u',
            'MC_STATUS_ARMED_QUERY_ONLY': '0x2000u',
        })

    def test_xml_object_and_pdo_contracts_match_the_pre_migration_esi(self):
        expected = {
            './/Dictionary': '9b8995e239f439d7fbd1467ef84dbd95f884fbe331e9303d17104b5f4dec4933',
            './/RxPdo': '0b821638565fbad3592fb6c81d826f597df70d0897e9709da49ab4a5efd0d233',
            './/TxPdo': 'ede08e6218a6d7982e3d339edcd92e4d91a43a4ab31cde8aba8bc7c34dfe68b5',
        }
        for path, checksum in expected.items():
            payload = [xml_contract(node) for node in self.xml.findall(path)]
            self.assertEqual(digest(json.dumps(payload, ensure_ascii=True, separators=(',', ':')).encode()), checksum)

    def test_all_three_assignments_retain_their_sizes_and_order(self):
        self.assertEqual(len(self.objects), 394)
        self.assertEqual(len(self.pdos), 77)
        for profile, expected in zip(self.schema['profiles'], ((736, 1024), (784, 1072), (544, 544))):
            sizes = []
            for direction in ('rx', 'tx'):
                indices = generator.profile_indices(profile, direction)
                self.assertEqual(indices, sorted(indices))
                sizes.append(sum(sum(generator.field_size(f) for f in self.pdos[i]['fields']) for i in indices))
            self.assertEqual(tuple(sizes), expected)

    def test_sii_preserves_identity_mailbox_fmmu_and_default_mapping(self):
        binary = self.generated[SII]
        self.assertEqual(len(binary), 16384)
        self.assertEqual(digest(binary[:128]), '47f050b1660c2a9db7d5df7d1b230a178e957b57512fab4e851327ac91ad711c')
        cats, end = categories(binary)
        self.assertEqual(list(cats), [10, 30, 40, 41, 50, 51, 60])
        self.assertLess(end // 2, 4096)
        self.assertEqual(set(binary[end:]), {255})
        for kind, checksum in {
            30: '0ddd184b8212f57ed6226ee820083b3b878e075fa1b2d9886b557f007d097808',
            40: '3e6f9aae16382bf563d8991b6da1b92213911f0dd5deea3ecaccf2f35a56794a',
            41: '13e44b7d3d505092b6bf44df869c75adff118400ed25a9f3f607c09a1e84f8b6',
            50: '14ace2e41f96a3394da90ce45ed9fb1d1fbbaadc6ae72ebc878f7927ce65fed3',
            51: 'd3636c6e35a47598e393cddf2dd6dc9af0638babf37ff0477be7ef4d4fb4f934',
        }.items():
            self.assertEqual(digest(cats[kind]), checksum, kind)
        image = self.generated['firmware/generated/ethercat/eeprom.h'].decode()
        self.assertEqual(bytes(int(x, 16) for x in re.findall(r'0x([0-9A-F]{2})', image)), binary)

    def test_sii_crc_known_vectors(self):
        self.assertEqual(generator.config_crc(bytes(14)), 0x30)
        self.assertEqual(generator.config_crc(bytes.fromhex('800c816600000000000000000000')), 0x62)

    def test_sii_dc_fields_and_strings_match_the_single_supported_mode(self):
        cats, _ = categories(self.generated[SII])
        string_data, strings, offset = cats[10], [], 1
        for _ in range(string_data[0]):
            length = string_data[offset]
            strings.append(string_data[offset + 1:offset + 1 + length].decode('ascii'))
            offset += length + 1
        self.assertEqual(strings, ['ECAT_Device', 'HPM_28AXIS', 'DC Sync0 (1-5 ms)'])
        self.assertEqual(len(cats[60]), 24)
        # ETG.2010/TI layout: AssignActivate at byte 14, factor at 16, names at 18/19.
        self.assertEqual(struct.unpack('<IIIhHhBB4s', cats[60]), (0, 0, 0, 0, 0x300, 1, 3, 3, bytes(4)))
        modes = self.xml.findall('.//Dc/OpMode')
        self.assertEqual(len(modes), 1)
        self.assertEqual(modes[0].findtext('AssignActivate'), '#x300')
        self.assertEqual(modes[0].find('CycleTimeSync0').get('Factor'), '1')

    def test_independent_sii_pdo_reader_agrees_with_esi(self):
        cats, _ = categories(self.generated[SII])
        codes = {'INT': 3, 'DINT': 4, 'USINT': 5, 'UINT': 6, 'UDINT': 7, 'REAL': 8, 'ULINT': 27}
        for kind, tag, sm in ((50, 'TxPdo', 3), (51, 'RxPdo', 2)):
            parsed, offset = [], 0
            while offset < len(cats[kind]):
                index, count, sync, reserved, name, flags = struct.unpack_from('<HBBBBH', cats[kind], offset)
                self.assertEqual((sync, reserved, name, flags), (sm, 0, 0, 0))
                offset += 8
                entries = []
                for _ in range(count):
                    idx, sub, name, typ, bits, flags = struct.unpack_from('<HBBBBH', cats[kind], offset)
                    self.assertEqual((name, flags), (0, 0))
                    entries.append((idx, sub, typ, bits))
                    offset += 8
                parsed.append((index, entries))
            expected = [(int(p.findtext('Index')[2:], 16), [
                (int(e.findtext('Index')[2:], 16), int(e.findtext('SubIndex')),
                 codes[e.findtext('DataType')], int(e.findtext('BitLen')))
                for e in p.findall('Entry')]) for p in self.xml.findall('.//' + tag) if 'Sm' in p.attrib]
            self.assertEqual(parsed, expected)

    def test_source_edits_affect_all_relevant_outputs(self):
        changed = copy.deepcopy(self.schema)
        changed['types']['mc_board_rx_t'][0]['default'] = 123
        objects, _ = generator.expand(changed)
        self.assertIn('.CycleSeq = 123', generator.storage(objects))
        self.assertNotEqual(generator.storage(objects).encode(), self.generated['firmware/generated/encos/encos_storage.inc'])
        changed['families'][0]['index'] = '0x20F0'
        objects, pdos = generator.expand(changed)
        self.assertIn(0x20F0, objects)
        self.assertIn('0x20F0', generator.gateway(changed, objects, pdos))
        xml = generator.esi((INPUTS / '设备模板.xml').read_bytes(), objects, pdos)
        self.assertIn(b'#x20F0', xml)

    def test_bad_schema_is_rejected(self):
        for mutate in (
            lambda s: s['families'].append(copy.deepcopy(s['families'][0])),
            lambda s: s['profiles'][0]['rx'].append('0x1A00'),
            lambda s: s['profiles'][0]['rx'].append('0x1600'),
            lambda s: s['types']['mc_board_rx_t'][0].update(default=2**32),
            lambda s: s['types']['mc_board_rx_t'][0].update(default=1.5),
            lambda s: s['types']['mc_board_rx_t'][0].update(name='bad-name'),
            lambda s: s.update(default_profile=-1),
        ):
            schema = copy.deepcopy(self.schema)
            mutate(schema)
            with self.assertRaises(ValueError):
                generator.expand(schema)

    def test_unsupported_or_oversized_sii_is_rejected(self):
        for mutate in (
            lambda d: setattr(d.find('./Eeprom/ByteSize'), 'text', '8192'),
            lambda d: setattr(d.find('./Dc/OpMode/AssignActivate'), 'text', '#x0'),
            lambda d: d.findall('Sm')[3].set('StartAddress', '#x1100'),
            lambda d: [p.set('Sm', '2') for p in d.findall('RxPdo')],
        ):
            xml = copy.deepcopy(self.xml)
            mutate(xml.find('./Descriptions/Devices/Device'))
            with self.assertRaises(ValueError):
                generator.sii(ET.tostring(xml), self.schema['sii'])

    def test_isolated_generated_header_storage_and_ssc_wrapper_compile(self):
        with tempfile.TemporaryDirectory(prefix='dcu-generated-compile-') as directory:
            work = Path(directory)
            for name, content in self.generated.items():
                target = work / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(content)
            includes = []
            for base, paths in ((work, ('firmware/generated/encos', 'firmware/generated/ethercat')),
                                (ROOT, ('tests/host/motor/stubs', 'firmware/cfg', 'firmware/ports',
                                        'firmware/comm/ethercat', 'third_party/beckhoff_ssc/Src'))):
                for path in paths:
                    includes += ['-I', str(base / path)]
            storage = work / 'storage.o'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Werror', *includes, '-c',
                            str(ROOT / 'firmware/comm/ethercat/encos_storage.c'), '-o', str(storage)], check=True, capture_output=True)
            cpp = work / 'main.cpp'
            cpp.write_text('#include "encos_interface.h"\nint main() { return mc_object_count != 394 || !mc_objects[0].data; }\n')
            subprocess.run(['c++', '-std=c++17', '-pedantic-errors', *includes, str(cpp), str(storage),
                            '-o', str(work / 'contract')], check=True, capture_output=True)
            subprocess.run([str(work / 'contract')], check=True)
            owner = work / 'owner.c'
            owner.write_text('#define _OBJD_\n#include "applInterface.h"\n#include "detail/ethercat_dictionary_owner.h"\n')
            subprocess.run(['cc', '-std=c11', '-Wall', '-Werror', *includes, '-fsyntax-only', str(owner)],
                           check=True, capture_output=True)


if __name__ == '__main__':
    unittest.main()
