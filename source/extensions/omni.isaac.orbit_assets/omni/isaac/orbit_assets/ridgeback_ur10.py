# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Ridgeback-Manipulation robots.

The following configurations are available:

* :obj:`RIDGEBACK_FRANKA_PANDA_CFG`: Clearpath Ridgeback base with Franka Emika arm

Reference: https://github.com/ridgeback/ridgeback_manipulation
"""

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.actuators import ImplicitActuatorCfg
from omni.isaac.orbit.assets.articulation import ArticulationCfg
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR

#
UR10_JOINT_NAMES = [
    "ur10_arm_shoulder_pan_joint",
    "ur10_arm_shoulder_lift_joint",
    "ur10_arm_elbow_joint",
    "ur10_arm_wrist_1_joint",
    "ur10_arm_wrist_2_joint",
    "ur10_arm_wrist_3_joint",
]

RIDGEBACK_UR10_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path="/home/tracy/Projects/mm_ws/src/mobile_manipulation_central/urdf/compiled/thing_pyb.urdf",
        fix_base=True,
        default_drive_type='velocity',
        default_drive_stiffness=2000,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            disable_gravity=True
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            # base
            "x_to_world_joint": 0.0,
            "y_to_x_joint": 0.0,
            "base_to_y_joint": 0.0,
            # ur10 arm
            "ur10_arm_shoulder_pan_joint": 0.0,
            "ur10_arm_shoulder_lift_joint": -0.569,
            "ur10_arm_elbow_joint": -2.810,
            "ur10_arm_wrist_1_joint": 0.0,
            "ur10_arm_wrist_2_joint": 3.037,
            "ur10_arm_wrist_3_joint": 0.741,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "base": ImplicitActuatorCfg(
            joint_names_expr=["x_to_world_joint","y_to_x_joint", "base_to_y_joint"],
            velocity_limit=100.0,
            effort_limit=1000.0,
            stiffness=0.0,
            damping=1e5,
        ),
        "ur10_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["ur10_arm_shoulder_.*"],
            effort_limit=87.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=40.0,
        ),
        "ur10_elbow": ImplicitActuatorCfg(
            joint_names_expr=["ur10_arm_elbow_.*"],
            effort_limit=87.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=40.0,
        ),
        "ur10_wrist": ImplicitActuatorCfg(
            joint_names_expr=["ur10_arm_wrist_.*"],
            effort_limit=87.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=40.0,
        ),
    },
)
"""Configuration of Franka arm with Franka Hand on a Clearpath Ridgeback base using implicit actuator models.

The following control configuration is used:

* Base: velocity control with damping
* Arm: position control with damping (contains default position offsets)
* Hand: mimic control

"""
