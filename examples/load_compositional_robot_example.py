"""
A script to collect a batch of human demonstrations.

The demonstrations can be played back using the `playback_demonstrations_from_hdf5.py` script.
"""

import argparse
import datetime
import json
import os
import shutil
import time
from glob import glob

import h5py
import numpy as np

import robosuite as suite
import robosuite.macros as macros
from robosuite.controllers import load_composite_controller_config
from robosuite.utils.input_utils import input2action
from robosuite.wrappers import DataCollectionWrapper, VisualizationWrapper

import mujoco
import mujoco.viewer

from robosuite.models.robots import *
from robosuite.robots import register_robot_class

import robosuite_models

"""
Here are two examples to define your own composition of robots. This is useful if you want to define robots in your own project codebase and do not mess up with the robosuite codebase. 

For more examples, see robosuite_models/robosuite/compositional.py
"""
@register_robot_class("WheeledRobot")
class UR5eOmron(UR5e):
    """
    Variant of Panda robot with mobile base. Currently serves as placeholder class.
    """

    @property
    def default_base(self):
        return "OmronMobileBase"

    @property
    def default_arms(self):
        return {"right": "UR5e"}

@register_robot_class("LeggedRobot")
class GR1SchunkSVHFloatingBody(GR1FloatingBody):
    """
    Variant of Panda robot with mobile base. Currently serves as placeholder class.
    """

    @property
    def default_gripper(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific gripper names
        """
        return {"right": "SchunkSvhRightHand", "left": "SchunkSvhLeftHand"}

if __name__ == "__main__":
    # Arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--directory",
        type=str,
        default=os.path.join(suite.models.assets_root, "demonstrations"),
    )
    parser.add_argument("--environment", type=str, default="Lift")
    parser.add_argument("--robots", nargs="+", type=str, default="Panda", help="Which robot(s) to use in the env")
    parser.add_argument(
        "--config", type=str, default="single-arm-opposed", help="Specified environment configuration if necessary"
    )
    parser.add_argument("--arm", type=str, default="right", help="Which arm to control (eg bimanual) 'right' or 'left'")
    parser.add_argument("--camera", type=str, default="agentview", help="Which camera to use for collecting demos")
    parser.add_argument(
        "--composite-controller", type=str, default=None, help="Choice of composite controller. Can be 'NONE' or 'WHOLE_BODY_IK'"
    )
    parser.add_argument(
        "--custom-controller-config", type=str, default=None, help="Choice of composite controller. Can be 'NONE' or 'WHOLE_BODY_IK'"
    )
    parser.add_argument("--device", type=str, default="keyboard")
    parser.add_argument("--pos-sensitivity", type=float, default=1.0, help="How much to scale position user inputs")
    parser.add_argument("--rot-sensitivity", type=float, default=1.0, help="How much to scale rotation user inputs")
    parser.add_argument(
        "--renderer",
        type=str,
        default="mujoco",
        help="Use the Nvisii viewer (Nvisii), OpenCV viewer (mujoco), or Mujoco's builtin interactive viewer (mjviewer)",
    )
    args = parser.parse_args()

    # Get controller config        
    composite_controller_config = load_composite_controller_config(custom_fpath=args.custom_controller_config, default_controller=args.composite_controller, robot=args.robots[0])

    # Create argument configuration
    config = {
        "env_name": args.environment,
        "robots": args.robots,
        "composite_controller_configs": composite_controller_config,
    }

    # Check if we're using a multi-armed environment and use env_configuration argument if so
    if "TwoArm" in args.environment:
        config["env_configuration"] = args.config

    # Create environment
    env = suite.make(
        **config,
        has_renderer=True,
        renderer=args.renderer,
        has_offscreen_renderer=False,
        render_camera=args.camera,
        ignore_done=True,
        use_camera_obs=False,
        reward_shaping=True,
        control_freq=20,
    )

    env.reset()
    env.step(np.zeros(env.robots[0].action_dim))

    m = env.sim.model._model
    d = env.sim.data._data
    mujoco.viewer.launch(m, d)
    exit()
