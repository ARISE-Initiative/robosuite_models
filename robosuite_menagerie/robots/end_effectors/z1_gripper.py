import numpy as np

from robosuite.models.grippers import register_eef
from robosuite.models.grippers.gripper_model import GripperModel

from robosuite_menagerie import menagerie_path_completion

@register_eef
class Z1Gripper(GripperModel):
    """
    Gripper for Franka's Panda (has two fingers).
    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(menagerie_path_completion("end_effectors/z1_gripper.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return np.array([0.0])

    @property
    def _important_geoms(self):
        return {"left_fingerpad": ["stator_col"], "right_fingerpad": ["mover_col"]}
