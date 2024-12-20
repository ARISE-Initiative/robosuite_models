import numpy as np
from robosuite.models.grippers import register_gripper
from robosuite.models.grippers.gripper_model import GripperModel

from robosuite_models import robosuite_model_path_completion


@register_gripper
class UMIGripper(GripperModel):
    """
    UMIGripper for the Arx5 arm
    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("grippers/umi_gripper.xml"), idn=idn)

    def format_action(self, action):
        assert len(action) == self.dof
        self.current_action = np.clip(self.current_action + np.array([1.0]) * self.speed * np.sign(action), -1.0, 1.0)
        return self.current_action

    @property
    def init_qpos(self):
        return np.array([-1.57, -1.57])

    @property
    def speed(self):
        return 0.2

    @property
    def _important_geoms(self):
        return {
            "right_fingerpad": ["right_finger_collision", "right_finger_visual"],
            "left_fingerpad": ["left_finger_visual", "left_finger_collision"],
        }
