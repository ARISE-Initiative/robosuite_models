import numpy as np
from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.robots import register_robot_class
from robosuite.utils.mjcf_utils import xml_path_completion

from robosuite_models import robosuite_model_path_completion


@register_robot_class("FixedBaseRobot")
class VX300S(ManipulatorModel):
    """
    VX300S is a single tabletop arm that is the unit for Aloha robot.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["right"]

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("robots/vx300s/robot.xml"), idn=idn)

    @property
    def default_base(self):
        return "RethinkMinimalMount"

    @property
    def default_gripper(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific gripper names
        """
        return {"right": "AlohaGripper"}
        # return {"right": "RethinkGripper", "left": "RethinkGripper"}

    @property
    def default_controller_config(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific default controller config names
        """
        return {"right": "default_aloha"}

    @property
    def init_qpos(self):
        """
        Since this is bimanual robot, returns [right, left] array corresponding to respective values

        Note that this is a pose such that the arms are half extended

        Returns:
            np.array: default initial qpos for the right, left arms
        """
        # [right, left]
        # Arms half extended
        return np.array([0, -0.840225, 0.847975, -0.1571, 1.53683, 0])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.29, 0, 0),
            "table": lambda table_length: (-0.22 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "single"

    @property
    def _eef_name(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific eef names
        """
        return {"right": "right_hand"}
