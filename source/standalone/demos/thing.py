# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates different single-arm manipulators.

.. code-block:: bash

    # Usage
    ./orbit.sh -p source/standalone/demos/arms.py

"""

from __future__ import annotations

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.orbit.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different single-arm manipulators.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import traceback

import carb
import omni.graph.core as og
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils import extensions

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.assets import Articulation
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.core_nodes.scripts.utils import set_target_prims
import omni.isaac.orbit.ros as ros_utils
from omni.isaac.orbit.sensors import CameraCfg, Camera
##
# Pre-defined configs
##
from omni.isaac.orbit_assets import RIDGEBACK_UR10_CFG  # isort:skip

extensions.enable_extension("omni.isaac.ros_bridge")
import rospy
from std_msgs.msg import Float64MultiArray


def design_scene() -> dict:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # -- Thing
    thing_cfg = RIDGEBACK_UR10_CFG.replace(prim_path="/World/Robot")
    robot = Articulation(cfg=thing_cfg)

    # Onboard Cameras
    # sensors
    camera_cfg = CameraCfg(
        prim_path="/Robot/cam",
        update_period=0.1,
        height=480,
        width=640,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
        ),
    )
    cam_base = Camera(camera_cfg.replace(prim_path="/World/Robot/arch_link/base_cam", 
                                         offset=CameraCfg.OffsetCfg(pos=(0.0, -0.5, 0.2), rot=(0.5, -0.5, 0.5, -0.5), convention="ros")))

    # return the scene information
    scene_entities = {"robot": robot, "sensors":[cam_base]}
    return scene_entities


def run_simulator(sim: sim_utils.SimulationContext, scene: dict):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    robot = scene["robot"]
    sensors = scene["sensors"]
    rospy.init_node("sim_isaac")
    robot_ros_interface = ros_utils.SimulatedMobileManipulatorROSInterface(robot_prim_path=robot.cfg.prim_path)
    while simulation_app.is_running():
        # reset
        if count % 200 == 0:
            # reset counters
            sim_time = 0.0
            count = 0
            # reset the scene entities
                # root state
            root_state = robot.data.default_root_state.clone()
            robot.write_root_state_to_sim(root_state)
            # set joint positions
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            robot.reset()
            print("[INFO]: Resetting robots state...")
        # apply random actions to the robots
        # generate random joint positions
        joint_vel_target = torch.randn_like(robot.data.joint_pos) * 0.1

        # apply action to the robot
        robot.set_joint_velocity_target(joint_vel_target)
        # write data to sim
        robot.write_data_to_sim()

        robot_ros_interface.publish_feedback(sim_time, robot.data.joint_pos[0], robot.data.joint_vel[0])
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1  
        # update buffers
        robot.update(sim_dt)


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg()
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene = design_scene()
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    try:
        # run the main execution
        main()
    except Exception as err:
        carb.log_error(err)
        carb.log_error(traceback.format_exc())
        raise
    finally:
        # close sim app
        simulation_app.close()
