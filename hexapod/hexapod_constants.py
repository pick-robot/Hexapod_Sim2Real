"""Hexapod robot constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.actuator import ElectricActuator, reflected_inertia
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

HEXAPOD_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "hexapod" / "xmls" / "hexapod.xml"
)
assert HEXAPOD_XML.exists()
HEXAPOD_ASSET_DIR = HEXAPOD_XML.parent / "assets"

LEG_NAMES = ("RF", "RM", "RB", "LF", "LM", "LB")
JOINT_NAMES = tuple(
  f"{leg}_{joint}" for leg in LEG_NAMES for joint in ("HAA", "HFE", "KFE")
)
FOOT_SITE_NAMES = LEG_NAMES
FOOT_GEOM_NAMES = tuple(f"{leg}_foot_collision" for leg in LEG_NAMES)
JOINT_AXIS_SIGNS = {
  "RF_HAA": 1,
  "RF_HFE": -1,
  "RF_KFE": -1,
  "RM_HAA": 1,
  "RM_HFE": -1,
  "RM_KFE": -1,
  "RB_HAA": 1,
  "RB_HFE": -1,
  "RB_KFE": 1,
  "LF_HAA": -1,
  "LF_HFE": 1,
  "LF_KFE": 1,
  "LM_HAA": -1,
  "LM_HFE": 1,
  "LM_KFE": 1,
  "LB_HAA": -1,
  "LB_HFE": 1,
  "LB_KFE": -1,
}


def get_spec() -> mujoco.MjSpec:
  """Load a fresh robot specification with exportable mesh buffers."""
  spec = mujoco.MjSpec.from_file(str(HEXAPOD_XML))
  for asset_path in sorted(HEXAPOD_ASSET_DIR.glob("*.STL")):
    spec.assets[f"assets/{asset_path.name}"] = asset_path.read_bytes()
  return spec


##
# Actuator config.
##

# Rotor inertia.
# 电机转子惯量 kgm^2
ROTOR_INERTIA = 0.00015

# Gearbox.减速比
HAA_GEAR_RATIO = 18
HFE_GEAR_RATIO = 18
KFE_GEAR_RATIO = 18

# 设置电机参数，反射惯量、速度限制、力矩限制
HAA_ACTUATOR = ElectricActuator(
  reflected_inertia=reflected_inertia(ROTOR_INERTIA, HAA_GEAR_RATIO),
  velocity_limit=16.44,
  effort_limit=30.0,
)
HFE_ACTUATOR = ElectricActuator(
  reflected_inertia=reflected_inertia(ROTOR_INERTIA, HFE_GEAR_RATIO),
  velocity_limit=16.44,
  effort_limit=30.0,
)
KFE_ACTUATOR = ElectricActuator(
  reflected_inertia=reflected_inertia(ROTOR_INERTIA, KFE_GEAR_RATIO),
  velocity_limit=16.44,
  effort_limit=30.0,
)

# 计算pd的参数
NATURAL_FREQ = 6 * 2.0 * 3.1415926535  # 6 Hz
# 阻尼比 明显过阻尼
DAMPING_RATIO = 1.1

# 计算kp和kd
# STIFFNESS_HAA = HAA_ACTUATOR.reflected_inertia * NATURAL_FREQ**2
# DAMPING_HAA = 2 * DAMPING_RATIO * HAA_ACTUATOR.reflected_inertia * NATURAL_FREQ

# STIFFNESS_HFE = HFE_ACTUATOR.reflected_inertia * NATURAL_FREQ**2
# DAMPING_HFE = 2 * DAMPING_RATIO * HFE_ACTUATOR.reflected_inertia * NATURAL_FREQ

# STIFFNESS_KFE = KFE_ACTUATOR.reflected_inertia * NATURAL_FREQ**2
# DAMPING_KFE = 2 * DAMPING_RATIO * KFE_ACTUATOR.reflected_inertia * NATURAL_FREQ

STIFFNESS_HAA = 100.0
DAMPING_HAA = 3.0
STIFFNESS_HFE = 100.0
DAMPING_HFE = 3.0
STIFFNESS_KFE = 100.0
DAMPING_KFE = 3.0

# 设置 PD 控制器。
HEXAPOD_HAA_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
  target_names_expr=(r".*_HAA",),
  stiffness=STIFFNESS_HAA,
  damping=DAMPING_HAA,
  effort_limit=HAA_ACTUATOR.effort_limit,
  armature=HAA_ACTUATOR.reflected_inertia,
)
HEXAPOD_HFE_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
  target_names_expr=(r".*_HFE",),
  stiffness=STIFFNESS_HFE,
  damping=DAMPING_HFE,
  effort_limit=HFE_ACTUATOR.effort_limit,
  armature=HFE_ACTUATOR.reflected_inertia,
)
HEXAPOD_KFE_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
  target_names_expr=(r".*_KFE",),
  stiffness=STIFFNESS_KFE,
  damping=DAMPING_KFE,
  effort_limit=KFE_ACTUATOR.effort_limit,
  armature=KFE_ACTUATOR.reflected_inertia,
)

##
# Keyframes.
##


# This lightly bent pose was solved against the compiled MJCF. The KFE signs
# deliberately differ between legs because the XML axes differ: all six shanks
# bend physically outward while the six foot spheres stay coplanar. Compared
# with q=0, it makes foot height 5--28 times more sensitive to KFE motion.
INIT_JOINT_POS = {
  "RF_HAA": 0.0,
  "RF_HFE": 0.0,
  "RF_KFE": 0.0,
  "RM_HAA": 0.0,
  "RM_HFE": 0.0,
  "RM_KFE": 0.0,
  "RB_HAA": 0.0,
  "RB_HFE": 0.0,
  "RB_KFE": 0.0,
  "LF_HAA": 0.0,
  "LF_HFE": 0.0,
  "LF_KFE": 0.0,
  "LM_HAA": 0.0,
  "LM_HFE": 0.0,
  "LM_KFE": 0.0,
  "LB_HAA": 0.0,
  "LB_HFE": 0.0,
  "LB_KFE": 0.0,
}

# At root z=0.313 m, every 0.023 m foot sphere starts about 0.30 mm above
# the plane. This avoids initial penetration while allowing immediate contact.
INIT_STATE = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.313),
  joint_pos=INIT_JOINT_POS,
  joint_vel={r".*": 0.0},
)

##
# Collision config.
##

_foot_regex = r"^(RF|RM|RB|LF|LM|LB)_foot_collision$"

# This disables all collisions except the feet.
# Furthermore, feet self collisions are disabled.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(_foot_regex,),
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
  solimp=(0.9, 0.95, 0.023),
)

# This enables all named collision geoms. Feet use six-dimensional contacts;
# non-foot collisions use normal-only contacts and remain frictionless.
_collision_regex = r".*_collision\d*"

FULL_COLLISION = CollisionCfg(
  geom_names_expr=(_collision_regex,),
  contype=1,
  conaffinity=1,
  # Harden all collision geoms.
  solref=(0.01, 1),
  # Configure feet colliders. Other colliders are frictionless (condim=1).
  condim={_foot_regex: 6, _collision_regex: 1},
  priority={_foot_regex: 1, ".*": 0},
  friction={_foot_regex: (1, 5e-3, 5e-4)},
)

##
# Final config.
##

HEXAPOD_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    HEXAPOD_HAA_ACTUATOR_CFG,
    HEXAPOD_HFE_ACTUATOR_CFG,
    HEXAPOD_KFE_ACTUATOR_CFG,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_hexapod_robot_cfg() -> EntityCfg:
  """Get a fresh hexapod robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=INIT_STATE,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=HEXAPOD_ARTICULATION,
  )


HEXAPOD_ACTION_SCALE: dict[str, float] = {}
for a in HEXAPOD_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    HEXAPOD_ACTION_SCALE[n] = 0.25


if __name__ == "__main__":
  from mjlab.entity import Entity

  robot = Entity(get_hexapod_robot_cfg())

  # viewer.launch(robot.spec.compile())

  print(f"STIFFNESS_HAA: {STIFFNESS_HAA}, DAMPING_HAA: {DAMPING_HAA}")
  print(f"STIFFNESS_HFE: {STIFFNESS_HFE}, DAMPING_HFE: {DAMPING_HFE}")
  print(f"STIFFNESS_KFE: {STIFFNESS_KFE}, DAMPING_KFE: {DAMPING_KFE}")
  print(HAA_ACTUATOR.reflected_inertia)
