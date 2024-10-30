"""
Here are two examples to define your own composition of robots. 
This is useful if you want to define robots in your own project codebase 
and do not mess up with the robosuite codebase. 

For more examples, see robosuite_models/robosuite/compositional.py
"""
import argparse

import mujoco
import mujoco.viewer
import numpy as np
import robosuite as suite
from robosuite.controllers import load_composite_controller_config
from robosuite.models.robots import *
from robosuite.robots import register_robot_class


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
    parser.add_argument("--environment", type=str, default="Lift")
    parser.add_argument(
        "--robots", nargs="+", type=str, default="PandaDexRHOmron", help="Which robot(s) to use in the env"
    )
    parser.add_argument(
        "--config", type=str, default="single-arm-opposed", help="Specified environment configuration if necessary"
    )
    parser.add_argument("--arm", type=str, default="right", help="Which arm to control (eg bimanual) 'right' or 'left'")
    parser.add_argument("--camera", type=str, default="agentview", help="Which camera to use for collecting demos")
    parser.add_argument(
        "--controller", type=str, default=None, help="Choice of composite controller. e.g. 'BASIC', 'WHOLE_BODY_IK'"
    )
    parser.add_argument("--device", type=str, default="keyboard")
    parser.add_argument(
        "--renderer",
        type=str,
        default="mjviewer",
        help="Use the Nvisii viewer (Nvisii), OpenCV viewer (mujoco), or Mujoco's builtin interactive viewer (mjviewer)",
    )
    args = parser.parse_args()

    # Get controller config
    composite_controller_config = load_composite_controller_config(controller=args.controller, robot=args.robots[0])

    # Create argument configuration
    config = {
        "env_name": args.environment,
        "robots": args.robots,
        "controller_configs": composite_controller_config,
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
    env.step(np.zeros(sum([robot.action_dim for robot in env.robots])))

    m = env.sim.model._model
    d = env.sim.data._data
    mujoco.viewer.launch(m, d)
    exit()
