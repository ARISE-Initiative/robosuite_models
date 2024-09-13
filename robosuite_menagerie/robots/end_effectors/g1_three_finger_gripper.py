"""
Dexterous hands for GR1 robot.
"""
import numpy as np

from robosuite.models.grippers import register_eef
from robosuite.models.grippers.gripper_model import GripperModel

from robosuite_menagerie import menagerie_path_completion

@register_eef
class G1ThreeFingerLeftGripper(GripperModel):
    """
    Three-finger left gripper  of G1 robot

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(menagerie_path_completion("end_effectors/g1_three_finger_left_gripper.xml"), idn=idn)

    def format_action(self, action):
        return action * np.array([0.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0])

    @property
    def init_qpos(self):
        return np.array([0.0] * 7)

    @property
    def speed(self):
        return 0.15

    @property
    def dof(self):
        return 7

@register_eef
class G1ThreeFingerRightGripper(GripperModel):
    """
    Three-finger right gripper of G1 robot

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(menagerie_path_completion("end_effectors/g1_three_finger_right_gripper.xml"), idn=idn)

    def format_action(self, action):
        return action * np.array([0.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0])

    @property
    def init_qpos(self):
        return np.array([0.0] * 7)

    @property
    def speed(self):
        return 0.15

    @property
    def dof(self):
        return 7
