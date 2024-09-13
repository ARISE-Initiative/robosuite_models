"""
Aloha Mount.
"""
import numpy as np

from robosuite.models.bases import register_base
from robosuite.models.bases.mount_model import MountModel

from robosuite_menagerie import menagerie_path_completion

@register_base
class AlohaMount(MountModel):
    """
    Mount officially used for Rethink's Baxter Robot. Includes only a wheeled pedestal.

    Args:
        idn (int or str): Number or some other unique identification string for this mount instance
    """

    def __init__(self, idn=0):
        super().__init__(menagerie_path_completion("mounts/aloha_mount.xml"), idn=idn)

    @property
    def top_offset(self):
        return np.array((0, 0, -0.062))

    @property
    def horizontal_radius(self):
        # TODO: This may be inaccurate; just a placeholder for now
        return 0.25