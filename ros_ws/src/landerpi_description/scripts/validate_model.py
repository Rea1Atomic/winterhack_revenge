#!/usr/bin/env python3
"""Validate the standalone model and binary STL resources (standard library only)."""
import argparse
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


def validate(package):
    model = package / 'urdf/landerpi_mecanum.urdf'
    text = model.read_text()
    assert all(token not in text for token in ('$(', '${', 'xacro:', '/home/')), 'Unresolved dependency'
    root = ET.fromstring(text)
    links = {x.attrib['name']: x for x in root.findall('link')}
    joints = {x.attrib['name']: x for x in root.findall('joint')}
    assert len(links) == len(root.findall('link')), 'Duplicate links'
    assert len(joints) == len(root.findall('joint')), 'Duplicate joints'
    parents = {}
    for name, joint in joints.items():
        parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
        assert parent in links and child in links, name
        assert child not in parents, f'Multiple parents: {child}'
        parents[child] = parent
        if joint.get('type') != 'fixed':
            axis = [float(v) for v in joint.find('axis').get('xyz').split()]
            assert abs(sum(v*v for v in axis)-1) < 1e-5, name
        mimic = joint.find('mimic')
        if mimic is not None:
            assert mimic.get('joint') in joints, name
    assert set(links)-set(parents) == {'base_footprint'}, 'Unexpected roots'
    for name in links:
        seen = set()
        current = name
        while current in parents:
            assert current not in seen, 'Cycle'
            seen.add(current)
            current = parents[current]
        assert current == 'base_footprint', name
    paths = set()
    for mesh in root.findall('.//mesh'):
        prefix = 'package://landerpi_description/'
        uri = mesh.get('filename')
        assert uri.startswith(prefix), uri
        path = package / uri[len(prefix):]
        assert path.is_file(), path
        paths.add(path)
    for path in paths:
        data = path.read_bytes()
        assert len(data) >= 84, path
        count = struct.unpack_from('<I', data, 80)[0]
        assert count > 0 and len(data) == 84+50*count, f'Invalid binary STL: {path}'
        for offset in range(84, len(data), 50):
            assert all(math.isfinite(v) for v in struct.unpack_from('<12f', data, offset)), path
    for el in root.iter():
        for key in ('xyz', 'rpy', 'rgba', 'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'):
            if key in el.attrib:
                assert all(math.isfinite(float(v)) for v in el.get(key).split()), key
    print(f'PASS: {len(links)} links, {len(joints)} joints, {len(paths)} valid STL assets; root=base_footprint')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-dir', type=Path, default=Path(__file__).resolve().parents[1])
    validate(parser.parse_args().package_dir.resolve())
