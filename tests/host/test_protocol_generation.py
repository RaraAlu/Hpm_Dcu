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
sys.path.insert(0, str(SOURCE))
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
        cls.legacy_schema = copy.deepcopy(cls.schema)
        del cls.legacy_schema['types']['mc_topology_t']
        del cls.legacy_schema['types']['mc_layout_identity_t']
        cls.legacy_schema['constants']['MC_INTERFACE_VERSION'] = '0x00010002u'
        del cls.legacy_schema['types']['mc_profile_update_t']
        del cls.legacy_schema['types']['mc_raw_switch_t']
        del cls.legacy_schema['types']['mc_raw_t']
        cls.legacy_schema['profiles'].pop()
        cls.legacy_schema['families'] = [row for row in cls.legacy_schema['families']
                                         if row['type'] in cls.legacy_schema['types']]
        cls.legacy_objects, cls.legacy_pdos = generator.expand(cls.legacy_schema)
        cls.legacy_xml = ET.fromstring(generator.esi((INPUTS / '设备模板.xml').read_bytes(),
                                                     cls.legacy_objects, cls.legacy_pdos))

    def test_generation_works_without_existing_outputs_and_is_reproducible(self):
        with tempfile.TemporaryDirectory(prefix='dcu-protocol-') as directory:
            root = Path(directory)
            source = root / 'tools/protocol'
            (source / 'cfg').mkdir(parents=True)
            for name in ('generate.py', 'generate_can_profile.py', 'check_gateway_contract.py',
                         'cfg/接口定义.json', 'cfg/设备模板.xml'):
                shutil.copy2(SOURCE / name, source / name)
            command = [sys.executable, str(source / 'generate.py'), '--profile', str(generator.DEFAULT_CAN_PROFILE)]
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

    def test_wire_c_tokens_match_the_profile_owned_can_contract(self):
        # Layout checksum remains from c8ea19f. Legacy configuration is reserved;
        # board and channel fields are read-only Profile observations.
        expected = {
            'encos/encos_interface.h': '2b74c842d03b6c4b96322ca0cfe2d2c15d5ffe211132e876d7c4df743f0fbf16',
            'encos/encos_storage.inc': 'b7fd8e1802ef1748cf7195e7980bce0cb068502372c92b9322e9a5f0c56ff968',
            'ethercat/gatewayObjects.h': '1fbde950eabe4005566bd14009edf11728c1c5bb10ae419219887a4de43fb6c9',
        }
        existing = {
            'encos/encos_interface.h': generator.header(self.legacy_schema),
            'encos/encos_storage.inc': generator.storage(self.legacy_objects),
            'ethercat/gatewayObjects.h': generator.gateway(self.legacy_schema, self.legacy_objects, self.legacy_pdos),
        }
        for name, checksum in expected.items():
            text = existing[name].replace('mc_sideband_t', 'mc_raw_t').replace('sideband.', 'raw.').replace('sideband size', 'raw size')
            text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
            text = re.sub(r'^#define MC_.*$', '', text, flags=re.M)
            text = re.sub(r'0x[0-9a-fA-F]+', lambda m: m[0].lower(), text)
            tokens = ' '.join(re.findall(r'\w+|[^\w\s]', text))
            self.assertEqual(digest(tokens.encode()), checksum, name)
        self.assertEqual(self.schema['constants'], {
            'MC_INTERFACE_VERSION': '0x00010003u', 'MC_AXIS_ARM': '0x0001u',
            'MC_AXIS_STOP': '0x0002u', 'MC_AXIS_RESET': '0x0004u',
            'MC_AXIS_START_MOTION': '0x0008u', 'MC_STATUS_QUERY_SOURCE': '0x1000u',
            'MC_STATUS_ARMED_QUERY_ONLY': '0x2000u',
        })

    def test_xml_has_read_only_can_settings_and_preserves_pdo_contracts(self):
        expected = {
            './/Dictionary': '237f9958cb31ff57e09d8b603f7dda01769eaa1cf82b0849ba5b2340e475adae',
            './/RxPdo': '0b821638565fbad3592fb6c81d826f597df70d0897e9709da49ab4a5efd0d233',
            './/TxPdo': 'ede08e6218a6d7982e3d339edcd92e4d91a43a4ab31cde8aba8bc7c34dfe68b5',
        }
        for path, checksum in expected.items():
            root = self.legacy_xml
            payload = [xml_contract(node) for node in root.findall(path)]
            self.assertEqual(digest(json.dumps(payload, ensure_ascii=True, separators=(',', ':')).encode()), checksum)

    def test_profile_update_adds_only_an_unmapped_mailbox_object(self):
        self.assertEqual({k: v for k, v in self.objects.items() if k not in (0x2100, 0x2101, 0x2102, 0x2103, 0x6401, 0x7301)}, self.legacy_objects)
        selection = self.objects[0x2103]['fields']
        self.assertEqual([generator.field_size(f) for f in selection], [88, 4, 4, 4, 8, 4])
        self.assertEqual([f['access'] for f in selection], ['wo', 'ro', 'ro', 'ro', 'ro', 'wo'])
        switch = self.objects[0x2101]['fields']
        self.assertEqual([generator.field_size(f) for f in switch], [41, 1, 1, 4, 4, 1, 4, 4])
        self.assertEqual([f['access'] for f in switch], ['wo', 'wo', 'ro', 'ro', 'rw', 'rw', 'ro', 'ro'])
        self.assertEqual({k: v for k, v in self.pdos.items() if k not in (0x1641, 0x1A42)}, self.legacy_pdos)
        self.assertNotEqual(self.generated[SII], generator.sii(ET.tostring(self.legacy_xml), self.schema['sii']))
        fields = self.objects[0x2100]['fields']
        self.assertEqual([generator.field_size(field) for field in fields], [36, 4, 2, 4, 4, 4, 4])
        self.assertEqual([field['access'] for field in fields], ['wo', 'wo', 'ro', 'ro', 'ro', 'ro', 'ro'])

    def test_reserved_objects_have_no_configuration_values_or_writable_fields(self):
        for name in ('routing', 'basic', 'range', 'gain', 'kt', 'action', 'maintenance'):
            for field in self.schema['types'][f'mc_{name}_t']:
                with self.subTest(type=name, field=field['name']):
                    self.assertEqual((field['access'], field['default']), ('ro', 0))

    def test_supported_object_catalog_retains_capacity_and_order(self):
        self.assertEqual(len(self.objects), 400)
        self.assertEqual(len(self.pdos), 79)
        self.assertEqual(len(self.schema['profiles']), 4)
        for profile, expected in zip(self.schema['profiles'], ((736, 1024), (784, 1072), (544, 544), (88, 88))):
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
        }.items():
            self.assertEqual(digest(cats[kind]), checksum, kind)
        self.assertEqual(tuple(struct.unpack_from('<H', cats[41], sm*8+2)[0] for sm in (2, 3)), (120, 128))
        image = self.generated['firmware/generated/ethercat/eeprom.h'].decode()
        self.assertEqual(bytes(int(x, 16) for x in re.findall(r'0x([0-9A-F]{2})', image)), binary)

    def test_sii_crc_known_vectors(self):
        self.assertEqual(generator.config_crc(bytes(14)), 0x30)
        self.assertEqual(generator.config_crc(bytes.fromhex('800c816600000000000000000000')), 0x62)

    def test_sii_dc_fields_and_strings_keep_the_boot_mode(self):
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

    def test_profile_compacts_independent_directions_without_reassigning_slots(self):
        fixture = ROOT / 'tests/host/motor/cfg/混挂电机验证配置.yaml'
        result = generator.outputs(profile_path=fixture, allow_encos_fixture=True)
        manifest = json.loads(result['protocol/ethercat/周期布局.json'])
        motor, mixed, raw, single = manifest['modes']
        self.assertEqual([(m['rx']['bytes'], m['tx']['bytes']) for m in (motor, mixed, raw, single)],
                         [(72, 144), (120, 192), (288, 288), (88, 88)])
        self.assertEqual([b['mapping'] for b in mixed['rx']['blocks']],
                         [0x1600, 0x1601, 0x1603, 0x162e, 0x1640])
        self.assertEqual([(b['mapping'], b['offset']) for b in mixed['tx']['blocks']],
                         [(0x1a00, 0), (0x1a01, 32), (0x1a03, 40), (0x1a20, 48),
                          (0x1a2e, 80), (0x1a2f, 112), (0x1a41, 144)])
        for d, tag in [('rx', 'RxPdo'), ('tx', 'TxPdo')]:
            xml = ET.fromstring(result[ESI])
            self.assertEqual([generator.number(p.findtext('Index')) for p in xml.findall('.//' + tag + '[@Sm]')],
                             [b['mapping'] for b in mixed[d]['blocks']])
        self.assertEqual(bytes.fromhex(manifest['sha256']), bytes(map(int, re.search(
            rb'od_2102 = \{\.Sha256 = \{([\d,]+)', result['firmware/generated/encos/encos_storage.inc'])[1].split(b','))))

    def test_route_and_equal_width_semantics_change_identity_even_at_identical_lengths(self):
        from generate_can_profile import load_yaml
        p = load_yaml(generator.DEFAULT_CAN_PROFILE)
        base = generator.cyclic_contract(self.schema, self.pdos, p)
        changed = copy.deepcopy(p)
        changed['devices'][0]['can_id'] += 1
        other = generator.cyclic_contract(self.schema, self.pdos, changed)
        self.assertEqual(base['modes'], other['modes'])
        self.assertNotEqual(base['sha256'], other['sha256'])
        changed = copy.deepcopy(p)
        changed['startup']['readiness_ttl_ms'] += 1
        other = generator.cyclic_contract(self.schema, self.pdos, changed)
        self.assertEqual(base['sha256'], other['sha256'])
        self.assertNotEqual(base['profile_sha256'], other['profile_sha256'])
        changed = copy.deepcopy(self.schema)
        fields = changed['types']['mc_axis_tx_t']
        a, b = (next(i for i, f in enumerate(fields) if f['name'] == name)
                for name in ('ActualPosition', 'ActualVelocity'))
        fields[a], fields[b] = fields[b], fields[a]
        other = generator.cyclic_contract(changed, generator.expand(changed)[1], p)
        self.assertEqual(base['modes'][1]['tx']['bytes'], other['modes'][1]['tx']['bytes'])
        self.assertNotEqual(base['sha256'], other['sha256'])

    def test_empty_and_full_motor_profiles_never_map_an_inactive_imu(self):
        from generate_can_profile import load_yaml
        p = load_yaml(generator.DEFAULT_CAN_PROFILE)
        p['devices'] = []
        p['channels'] = [dict(p['channels'][0], bus=n) for n in range(4)]
        def sizes():
            image = generator.cyclic_contract(self.schema, self.pdos, p)['modes'][1]
            self.assertNotIn(0x1a40, [b['mapping'] for b in image['tx']['blocks']])
            return image['rx']['bytes'], image['tx']['bytes']
        self.assertEqual(sizes(), (112, 112))
        p['devices'] = [dict(id=f'axis_{bus}_{n}', protocol='encos', type='7216', bus=bus, can_id=n+1)
                        for bus in range(4) for n in range(7)]
        self.assertEqual(sizes(), (784, 1008))

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

    def test_rk_rejects_wrong_digest_entries_direction_and_registered_offsets(self):
        with tempfile.TemporaryDirectory() as work:
            exe = Path(work) / 'verify-layout'
            target = ROOT / 'tests/host/ethercat_data'
            subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-I', str(target / 'layout_stubs'), '-I', str(ROOT / 'protocol/ethercat'),
                            str(target / 'test_profile_layout.cpp'), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_hot_rk_selection_uses_catalogue_slots_and_validates_live_fields_before_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            bench = ROOT / 'firmware/app/profile/cfg/五轴台架配置.yaml'
            header = 'protocol/ethercat/dcu_profile_pdo.hpp'
            (work / 'dcu_profile_pdo.hpp').write_bytes(generator.outputs(profile_path=bench)[header])
            target = ROOT / 'tests/host/ethercat_data'
            exe = work / 'hot-selection'
            subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(work),
                            '-I', str(target / 'layout_stubs'), '-I', str(ROOT / 'protocol/ethercat'),
                            str(target / 'test_hot_selection.cpp'), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_yaml_hot_selection_accepts_subsets_not_route_or_safety_changes(self):
        from apply_cyclic_profile import selection
        base = generator.DEFAULT_CAN_PROFILE.with_name('五轴台架配置.yaml')
        from generate_can_profile import load_yaml
        catalogue = load_yaml(base)
        desired = copy.deepcopy(catalogue)
        desired['devices'].pop(2)
        expected = ['0xc', 'hra55h100_can3', 'encos_can2_1', 'encos_can2_3', 'encos_can2_4']
        self.assertEqual(selection(catalogue, desired), expected)
        for change in (lambda p: p['devices'][1].update(can_id=2),
                       lambda p: p['devices'][1]['limits'].update(current_dA=2),
                       lambda p: p['devices'].reverse(),
                       lambda p: p['startup'].update(revision=10)):
            modified = copy.deepcopy(desired)
            change(modified)
            with self.assertRaises(ValueError):
                selection(catalogue, modified)

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

    def test_sm2_mode_changes_only_the_esi_selection(self):
        xml = copy.deepcopy(self.xml)
        dc = xml.find('./Descriptions/Devices/Device/Dc')
        self.assertEqual([mode.findtext('AssignActivate') for mode in dc], ['#x300', '#x0'])
        no_pulses = dc[1]
        ET.SubElement(no_pulses, 'CycleTimeSync0').text = '1000000'
        with self.assertRaises(ValueError):
            generator.sii(ET.tostring(xml), self.schema['sii'])
        dc.remove(no_pulses)
        self.assertEqual(generator.sii(ET.tostring(xml), self.schema['sii']), self.generated[SII])

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
            cpp.write_text('#include "encos_interface.h"\nint main() { return mc_object_count != 400 || !mc_objects[0].data; }\n')
            subprocess.run(['c++', '-std=c++17', '-pedantic-errors', *includes, str(cpp), str(storage),
                            '-o', str(work / 'contract')], check=True, capture_output=True)
            subprocess.run([str(work / 'contract')], check=True)
            owner = work / 'owner.c'
            owner.write_text('#define _OBJD_\n#include "applInterface.h"\n#include "detail/ethercat_dictionary_owner.h"\n')
            subprocess.run(['cc', '-std=c11', '-Wall', '-Werror', *includes, '-fsyntax-only', str(owner)],
                           check=True, capture_output=True)


if __name__ == '__main__':
    unittest.main()
