#!/usr/bin/env python3
"""Build a MuJoCo scene from the workspace's authoritative URDF and SRDF."""
import argparse
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def stl_to_obj(source, target):
    # Lossless geometry conversion: MuJoCo's STL decoder limits triangle count.
    data = source.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + count * 50:
        raise ValueError(f'Expected binary STL: {source}')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
                      ('attribute', '<u2')])
    triangles = np.frombuffer(data, dtype=dtype, offset=84, count=count)
    vertices, indices = np.unique(
        triangles['vertices'].reshape(-1, 3), axis=0, return_inverse=True)
    with target.open('w') as stream:
        np.savetxt(stream, vertices, fmt='v %.9g %.9g %.9g')
        np.savetxt(stream, indices.reshape(-1, 3) + 1, fmt='f %d %d %d')


def generate(description, moveit, output):
    output.mkdir(parents=True, exist_ok=True)
    meshes = output / 'meshes'
    meshes.mkdir(exist_ok=True)
    urdf = ET.parse(description / 'urdf/test_robot.urdf').getroot()
    for mesh in urdf.findall('.//mesh'):
        filename = Path(mesh.get('filename')).name
        target = meshes / (Path(filename).stem + '.obj')
        if not target.exists():
            stl_to_obj(description / 'meshes' / filename, target)
        mesh.set('filename', str(target.resolve()))
    ET.SubElement(ET.SubElement(urdf, 'mujoco'), 'compiler',
                  discardvisual='false', fusestatic='false', balanceinertia='true')
    # Add the coupling explicitly below, consistently across MuJoCo versions.
    mimics = []
    for joint in urdf.findall('joint'):
        mimic = joint.find('mimic')
        if mimic is not None:
            mimics.append((joint.get('name'), dict(mimic.attrib)))
            joint.remove(mimic)
    model = mujoco.MjModel.from_xml_string(ET.tostring(urdf, encoding='unicode'))
    filename = output / 'test_robot.xml'
    mujoco.mj_saveLastXML(str(filename), model)
    root = ET.parse(filename).getroot()
    root.find('compiler').set('meshdir', 'meshes')
    for mesh in root.findall('asset/mesh'):
        mesh.set('file', Path(mesh.get('file')).name)
    ET.SubElement(root, 'option', timestep='0.001', integrator='implicitfast',
                  gravity='0 0 -9.81', iterations='100')
    default = root.find('default')
    if default is None:
        default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'joint', damping='0.05', armature='0.001')
    world = root.find('worldbody')
    ET.SubElement(world, 'light', pos='0 -1 2', dir='0 0 -1', directional='true')
    # The base mesh bottom is at z=0 in the URDF mounting frame.
    ET.SubElement(world, 'geom', name='floor', type='plane', pos='0 0 -0.01',
                  size='2 2 0.1', rgba='0.25 0.28 0.32 1')
    contact = ET.SubElement(root, 'contact')
    bodies = {b.get('name') for b in root.findall('.//body')}
    for pair in ET.parse(moveit / 'config/test_robot.srdf').getroot().findall('disable_collisions'):
        if pair.get('link1') in bodies and pair.get('link2') in bodies:
            ET.SubElement(contact, 'exclude', body1=pair.get('link1'), body2=pair.get('link2'))
    equality = ET.SubElement(root, 'equality')
    for name, mimic in mimics:
        ET.SubElement(equality, 'joint', joint1=name, joint2=mimic['joint'],
                      polycoef=f"{mimic.get('offset', '0')} {mimic.get('multiplier', '1')} 0 0 0",
                      solref='0.002 1')
    actuator = ET.SubElement(root, 'actuator')
    gains = {'joint1': (600, 30), 'joint2': (1200, 60), 'joint3': (800, 40),
             'joint4': (400, 20), 'joint5': (300, 15), 'joint6': (200, 10),
             'joint8': (10000, 100)}
    for joint in urdf.findall('joint'):
        name = joint.get('name')
        if name not in gains:
            continue
        limit = joint.find('limit')
        kp, kv = gains[name]
        effort = float(limit.get('effort'))
        ET.SubElement(actuator, 'position', name=name + '_position', joint=name,
                      kp=str(kp), kv=str(kv), ctrllimited='true',
                      ctrlrange=f"{limit.get('lower')} {limit.get('upper')}",
                      forcelimited='true', forcerange=f'{-effort} {effort}')
    ET.indent(root)
    ET.ElementTree(root).write(filename, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(filename))
    print(f'Generated {filename}: {model.nq} joints, {model.nu} actuators')
    return filename


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description', type=Path, required=True)
    parser.add_argument('--moveit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    generate(args.description, args.moveit, args.output)
