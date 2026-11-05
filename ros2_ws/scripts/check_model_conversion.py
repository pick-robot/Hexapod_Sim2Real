#!/usr/bin/env python3
"""Compare the generated URDF against compiled MJCF, including nonzero poses.

Usage: Python-with-mujoco check_model_conversion.py /path/to/expanded.urdf
"""

import hashlib
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from import_mjcf import JOINT_NAMES, PACKAGE, SOURCE, read_constants, rotation


def transform(origin):
    xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    roll, pitch, yaw = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    result[:3, 3] = xyz
    return result


def main():
    urdf = ET.parse(sys.argv[1]).getroot()
    model = mujoco.MjModel.from_xml_path(str(SOURCE / 'xmls' / 'hexapod.xml'))
    data = mujoco.MjData(model)
    limits, initial = read_constants()
    links = {l.get('name'): l for l in urdf.findall('link')}
    joints = urdf.findall('joint')
    assert len(links) == model.nbody  # world replaced by massless base_link
    assert len(urdf.findall('link/collision')) == 62  # 18 capsules * 3 + 2 cylinders + 6 feet
    for b in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        inertial = links[name].find('inertial')
        assert abs(float(inertial.find('mass').get('value')) - model.body_mass[b]) < 1e-12
        frame = transform(inertial.find('origin'))
        np.testing.assert_allclose(frame[:3, 3], model.body_ipos[b], atol=1e-12)
        diagonal = np.diag([float(inertial.find('inertia').get(k)) for k in ('ixx', 'iyy', 'izz')])
        expected = rotation(model.body_iquat[b]) @ np.diag(model.body_inertia[b]) @ rotation(model.body_iquat[b]).T
        np.testing.assert_allclose(frame[:3, :3] @ diagonal @ frame[:3, :3].T, expected, atol=1e-12)
    for j in joints:
        if j.get('type') != 'revolute':
            continue
        name = j.get('name')
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        np.testing.assert_allclose(np.fromstring(j.find('axis').get('xyz'), sep=' '), model.jnt_axis[jid], atol=1e-12)
        np.testing.assert_allclose([float(j.find('limit').get(k)) for k in ('lower', 'upper')], model.jnt_range[jid], atol=1e-12)
        assert float(j.find('limit').get('effort')) == limits[name.split('_')[-1]]['effort_limit']
    rng = np.random.default_rng(42)
    for sample in range(21):
        q = dict(zip(JOINT_NAMES, np.zeros(18) if sample == 0 else rng.uniform(-0.5, 0.5, 18)))
        data.qpos[:] = model.qpos0
        data.qpos[:3] = initial
        for name, value in q.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[jid]] = value
        mujoco.mj_forward(model, data)
        base = np.eye(4)
        base[:3, 3] = initial
        world = {'base_link': base}
        for j in joints:
            pose = transform(j.find('origin')) if j.find('origin') is not None else np.eye(4)
            if j.get('type') == 'revolute':
                axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                quat = np.r_[np.cos(q[j.get('name')]/2), axis*np.sin(q[j.get('name')]/2)]
                pose[:3, :3] = pose[:3, :3] @ rotation(quat)
            world[j.find('child').get('link')] = world[j.find('parent').get('link')] @ pose
        for b in range(1, model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
            np.testing.assert_allclose(world[name][:3, 3], data.xpos[b], atol=1e-10)
            np.testing.assert_allclose(world[name][:3, :3], data.xmat[b].reshape(3, 3), atol=1e-10)
        for gid in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            if not name:  # visuals are validated below from original mesh transforms
                continue
            kind = model.geom_type[gid]
            urdf_name = name.replace('_foot_collision', '_FOOT_collision')
            matched = 0
            for link_name, link in links.items():
                for c in link.findall('collision'):
                    c_name = c.get('name')
                    if c_name != urdf_name and not c_name.startswith(urdf_name + '_'):
                        continue
                    matched += 1
                    actual = world[link_name] @ transform(c.find('origin'))
                    expected_position = data.geom_xpos[gid].copy()
                    if '_cap_' in c_name:
                        sign = int(c_name.rsplit('_', 1)[-1])
                        expected_position += sign * model.geom_size[gid, 1] * data.geom_xmat[gid].reshape(3, 3)[:, 2]
                    np.testing.assert_allclose(actual[:3, 3], expected_position, atol=1e-10)
                    shape = c.find('geometry')[0]
                    assert abs(float(shape.get('radius')) - model.geom_size[gid, 0]) < 1e-12
                    if shape.tag == 'cylinder':
                        assert abs(float(shape.get('length')) - 2*model.geom_size[gid, 1]) < 1e-12
                        np.testing.assert_allclose(actual[:3, 2], data.geom_xmat[gid].reshape(3, 3)[:, 2], atol=1e-10)
            assert matched == (3 if kind == mujoco.mjtGeom.mjGEOM_CAPSULE else 1), (name, matched)
        # Raw STL visuals must undo the mesh centering MuJoCo bakes into geoms.
        for b in range(1, model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
            visual_ids = [g for g in range(model.ngeom) if model.geom_bodyid[g] == b and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
            visuals = links[name].findall('visual')
            assert len(visual_ids) == len(visuals)
            for gid, visual in zip(visual_ids, visuals):
                mesh_id = model.geom_dataid[gid]
                mesh_frame = np.eye(4)
                mesh_frame[:3, :3] = rotation(model.mesh_quat[mesh_id])
                mesh_frame[:3, 3] = model.mesh_pos[mesh_id]
                expected = np.eye(4)
                expected[:3, :3] = data.geom_xmat[gid].reshape(3, 3)
                expected[:3, 3] = data.geom_xpos[gid]
                np.testing.assert_allclose(world[name] @ transform(visual.find('origin')) @ mesh_frame, expected, atol=1e-9)
    for asset in (SOURCE / 'xmls' / 'assets').glob('*.STL'):
        assert hashlib.sha256(asset.read_bytes()).digest() == hashlib.sha256((PACKAGE / 'meshes' / asset.name).read_bytes()).digest()
    print('PASS: 25 body masses/inertias, 18 axes/limits, all visuals and collision poses match MJCF in 21 poses; all STL hashes match.')


if __name__ == '__main__':
    main()
