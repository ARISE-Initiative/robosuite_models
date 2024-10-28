import numpy as np
from robosuite.models.bases import register_base
from robosuite.models.bases.leg_base_model import LegBaseModel

from robosuite_models import robosuite_model_path_completion


@register_base
class Go2(LegBaseModel):
    """
    Go2 Base Class.

    Args:
        idn (int or str): Number or some other unique identification string for this mount instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("robots/go2/robot.xml"), idn=idn)

    @property
    def top_offset(self):
        return np.array((0, 0, 0))

    @property
    def horizontal_radius(self):
        return 0.1

    @property
    def init_qpos(self):
        return np.array([0.0, 0.9, -1.8] * 4)


@register_base
class Go2Floating(LegBaseModel):
    """
    Variant of Go2 robot with floating base.

    Args:
        idn (int or str): Number or some other unique identification string for this mount instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("robots/go2/robot.xml"), idn=idn)

        self._remove_joint_actuation("leg")
        self._remove_free_joint()

        self._add_mobile_joint()

    @property
    def top_offset(self):
        return np.array((0, 0, 0))

    @property
    def horizontal_radius(self):
        return 0.1

    @property
    def init_qpos(self):
        return np.array([])
