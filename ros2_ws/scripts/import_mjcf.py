#!/usr/bin/env python3
"""Regenerate the Gazebo description from this repository's hexapod MJCF.

Run with a Python environment providing mujoco and numpy. Gazebo itself does
not need either dependency. Never import hexapod_constants (it imports MJLab).
"""

import ast
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


WORKSPACE = Path(__file__).resolve().parents[1]
REPOSITORY = WORKSPACE.parent
SOURCE = REPOSITORY / 'hexapod'
PACKAGE = WORKSPACE / 'src' / 'hexapod_description'
JOINT_NAMES = [f'{leg}_{axis}' for leg in ('RF', 'RM', 'RB', 'LF', 'LM', 'LB')
               for axis in ('HAA', 'HFE', 'KFE')]
XACRO = 'http://www.ros.org/wiki/xacro'
ET.register_namespace('xacro', XACRO)


def numbers(values):
    return ' '.join(format(float(v), '.16g') for v in values)


def rotation(quaternion):
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, np.asarray(quaternion, dtype=float))
    return matrix.reshape(3, 3)


def rpy(quaternion):
    matrix = rotation(quaternion)
    pitch = np.arctan2(-matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0]))
    if np.hypot(matrix[0, 0], matrix[1, 0]) < 1e-10:
        roll, yaw = np.arctan2(-matrix[1, 2], matrix[1, 1]), 0.0
    else:
        roll, yaw = np.arctan2(matrix[2, 1], matrix[2, 2]), np.arctan2(matrix[1, 0], matrix[0, 0])
    return numbers((roll, pitch, yaw))


def origin(parent, position, quaternion=(1, 0, 0, 0)):
    ET.SubElement(parent, 'origin', xyz=numbers(position), rpy=rpy(quaternion))


def read_constants():
    tree = ast.parse((SOURCE / 'hexapod_constants.py').read_text())
    assignments = {node.targets[0].id: node.value for node in tree.body
                   if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)}
    limits = {}
    for axis in ('HAA', 'HFE', 'KFE'):
        call = assignments[f'{axis}_ACTUATOR']
        limits[axis] = {k.arg: ast.literal_eval(k.value) for k in call.keywords
                        if k.arg in ('effort_limit', 'velocity_limit')}
    initial = assignments['INIT_STATE']
    position = ast.literal_eval(next(k.value for k in initial.keywords if k.arg == 'pos'))
    assert ast.literal_eval(assignments['INIT_JOINT_POS']) == dict.fromkeys(JOINT_NAMES, 0.0)
    return limits, position


def add_collisions(link, model, body_id):
    for geom_id in range(model.ngeom):
        if model.geom_bodyid[geom_id] != body_id or not (
                model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]):
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        position, quat, size = model.geom_pos[geom_id], model.geom_quat[geom_id], model.geom_size[geom_id]
        kind = model.geom_type[geom_id]

        def collider(suffix, pos, shape, **attributes):
            collision = ET.SubElement(link, 'collision', name=name + suffix)
            origin(collision, pos, quat)
            ET.SubElement(ET.SubElement(collision, 'geometry'), shape, **attributes)

        if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            # Exact geometric union; Classic URDF has no capsule primitive.
            collider('_cylinder', position, 'cylinder', radius=str(size[0]), length=str(2*size[1]))
            axis = rotation(quat)[:, 2]
            for sign in (-1, 1):
                collider(f'_cap_{sign:+d}', position + sign*size[1]*axis, 'sphere', radius=str(size[0]))
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            collider('', position, 'cylinder', radius=str(size[0]), length=str(2*size[1]))
        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            collider('', position, 'sphere', radius=str(size[0]))
        else:
            raise ValueError(f'Unsupported collision {name}: {kind}')


def generate():
    xml_path = SOURCE / 'xmls' / 'hexapod.xml'
    raw = ET.parse(xml_path).getroot()
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    limits, initial_position = read_constants()
    assert model.nbody == 26 and model.njnt == 19
    assert [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            for j in range(1, model.njnt)] == JOINT_NAMES
    output_path = PACKAGE / 'urdf' / 'hexapod.urdf.xacro'
    root = ET.Element('robot', name='hexapod')
    for name, default in [('mesh_prefix', 'package://hexapod_description/meshes'),
                          ('use_ros2_control', 'false'), ('controllers_file', '')]:
        ET.SubElement(root, f'{{{XACRO}}}arg', name=name, default=default)
    efforts = {limit['effort_limit'] for limit in limits.values()}
    assert len(efforts) == 1, 'Plugin template expects a common effort limit'
    ET.SubElement(root, f'{{{XACRO}}}property', name='joint_effort_limit', value=str(efforts.pop()))
    ET.SubElement(root, f'{{{XACRO}}}include', filename='gazebo_plugins.xacro')
    ET.SubElement(root, 'link', name='base_link')
    raw_bodies = {body.get('name'): body for body in raw.findall('.//body')}
    assets = {mesh.get('name'): mesh for mesh in raw.findall('asset/mesh')}
    for body_id in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        link = ET.SubElement(root, 'link', name=name)
        inertial = ET.SubElement(link, 'inertial')
        # Compiled inertia honors balanceinertia=true and its principal frame.
        origin(inertial, model.body_ipos[body_id], model.body_iquat[body_id])
        ET.SubElement(inertial, 'mass', value=str(model.body_mass[body_id]))
        ix, iy, iz = model.body_inertia[body_id]
        ET.SubElement(inertial, 'inertia', ixx=str(ix), iyy=str(iy), izz=str(iz),
                      ixy='0', ixz='0', iyz='0')
        for geom in raw_bodies[name].findall("geom[@class='visual']"):
            # Use original XML poses for STL visuals: compiled mesh geoms
            # include MuJoCo's mesh centering/principal-axis transform.
            asset = assets[geom.get('mesh')]
            assert set(asset.attrib) <= {'name', 'file', 'class', 'scale'}
            assert set(geom.attrib) <= {'class', 'mesh', 'rgba', 'pos', 'quat'}
            visual = ET.SubElement(link, 'visual')
            origin(visual, np.fromstring(geom.get('pos', '0 0 0'), sep=' '),
                   np.fromstring(geom.get('quat', '1 0 0 0'), sep=' '))
            ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh',
                          filename='$(arg mesh_prefix)/' + asset.get('file'),
                          scale=asset.get('scale', '1 1 1'))
            ET.SubElement(ET.SubElement(visual, 'material', name=geom.get('mesh') + '_material'),
                          'color', rgba=geom.get('rgba', '1 1 1 1'))
        add_collisions(link, model, body_id)
        parent_id = model.body_parentid[body_id]
        joint_ids = list(range(model.body_jntadr[body_id],
                               model.body_jntadr[body_id] + model.body_jntnum[body_id]))
        hinges = [j for j in joint_ids if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
        assert len(hinges) <= 1
        if hinges:
            j = hinges[0]
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            assert np.allclose(model.jnt_pos[j], 0), 'Only origin-centered hinges supported'
            joint = ET.SubElement(root, 'joint', name=joint_name, type='revolute')
            origin(joint, model.body_pos[body_id], model.body_quat[body_id])
            ET.SubElement(joint, 'axis', xyz=numbers(model.jnt_axis[j]))
            limit = limits[joint_name.split('_')[-1]]
            ET.SubElement(joint, 'limit', lower=str(model.jnt_range[j, 0]), upper=str(model.jnt_range[j, 1]),
                          effort=str(limit['effort_limit']), velocity=str(limit['velocity_limit']))
            dof = model.jnt_dofadr[j]
            ET.SubElement(joint, 'dynamics', damping=str(model.dof_damping[dof]),
                          friction=str(model.dof_frictionloss[dof]))
        else:
            joint = ET.SubElement(root, 'joint', name=name + '_fixed', type='fixed')
            origin(joint, (0, 0, 0) if parent_id == 0 else model.body_pos[body_id],
                   model.body_quat[body_id])
        ET.SubElement(joint, 'parent', link='base_link' if parent_id == 0 else
                      mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id))
        ET.SubElement(joint, 'child', link=name)
        # MJLab FULL_COLLISION uses condim=1 for non-foot collisions. Gazebo
        # friction is link-wide, so carry foot spheres on separate fixed links.
        surface = ET.SubElement(root, 'gazebo', reference=name)
        for tag in ('mu1', 'mu2'):
            ET.SubElement(surface, tag).text = '0'
        ET.SubElement(surface, 'selfCollide').text = 'false'

    # Move each sphere to its existing fixed FOOT frame. This preserves the
    # geometric pose and allows foot friction without adding mass or links.
    for leg in ('RF', 'RM', 'RB', 'LF', 'LM', 'LB'):
        shank = root.find(f"link[@name='{leg}_SHANK']")
        foot_collision = shank.find(f"collision[@name='{leg}_foot_collision']")
        shank.remove(foot_collision)
        foot_name = leg + '_FOOT'
        # SDFormat's URDF fixed-joint reducer associates these extensions by
        # the link-prefixed collision name; otherwise the surface is lost.
        foot_collision.set('name', foot_name + '_collision')
        foot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, foot_name)
        sphere_origin = foot_collision.find('origin')
        sphere_position = np.fromstring(sphere_origin.get('xyz'), sep=' ')
        sphere_origin.set('xyz', numbers(rotation(model.body_quat[foot_id]).T @
                                         (sphere_position - model.body_pos[foot_id])))
        sphere_origin.set('rpy', '0 0 0')
        root.find(f"link[@name='{foot_name}']").append(foot_collision)
        surface = root.find(f"gazebo[@reference='{foot_name}']")
        surface.find('mu1').text = '1'
        surface.find('mu2').text = '1'
        for tag, value in [('kp', '200000'), ('kd', '50'),
                           ('minDepth', '0.001'), ('maxVel', '0.1')]:
            ET.SubElement(surface, tag).text = value
        fixed = ET.SubElement(root, 'gazebo', reference=foot_name + '_fixed')
        ET.SubElement(fixed, 'preserveFixedJoint').text = 'true'

    # 附加传感器由独立 Xacro 维护，避免重新转换 MJCF 时丢失安装配置。
    # 放在机器人连杆之后，也便于按父子顺序检查 URDF 的坐标树。
    ET.SubElement(root, f'{{{XACRO}}}include', filename='airy_lidar.xacro')
    ET.indent(root, space='  ')
    output_path.write_text('<?xml version="1.0"?>\n<!-- Generated by ros2_ws/scripts/import_mjcf.py; edit the source MJCF, then regenerate. -->\n'
                           + ET.tostring(root, encoding='unicode') + '\n')
    for mesh in assets.values():
        shutil.copyfile(xml_path.parent / 'assets' / mesh.get('file'), PACKAGE / 'meshes' / mesh.get('file'))
    report = {
        'source': 'hexapod/xmls/hexapod.xml',
        'source_sha256': hashlib.sha256(xml_path.read_bytes()).hexdigest(),
        'constants_sha256': hashlib.sha256((SOURCE / 'hexapod_constants.py').read_bytes()).hexdigest(),
        'mujoco_version': mujoco.__version__, 'body_mass_kg': float(model.body_mass.sum()),
        'initial_position': initial_position, 'joint_names': JOINT_NAMES,
        'limits': limits, 'capsules_split_into_cylinder_and_spheres': True,
    }
    (PACKAGE / 'model_info.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    generate()
