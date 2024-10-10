from robosuite.robots import register_robot_class
from robosuite.models.robots import *
from robosuite_menagerie.robots import *

@register_robot_class("WheeledRobot")
class VX300SMobile(VX300S):
    """
    Variant of VX300S robot with mobile base. Currently serves as placeholder class.
    """

    @property
    def default_base(self):
        return "OmronMobileBase"

    @property
    def default_arms(self):
        return {"right": "VX300S"}

@register_robot_class("LeggedRobot")
class B1Z1(Z1):
    """
    Variant of VX300S robot with mobile base. Currently serves as placeholder class.
    """

    @property
    def default_base(self):
        return "B1"

    @property
    def default_arms(self):
        return {"right": "Z1"}

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.8, -0.1, 0.65),
            "empty": (-0.8, 0, 0.65),
            "table": lambda table_length: (-0.55 - table_length / 2, 0.9, 0.65),
        }

@register_robot_class("LeggedRobot")
class B1Z1Floating(B1Z1):
    """
    Variant of VX300S robot with mobile base. Currently serves as placeholder class.
    """

    @property
    def default_base(self):
        return "FloatingLeggedBase"

@register_robot_class("LeggedRobot")
class Go2Arx5(Arx5):
    """
    Go2 robot with Arx5 arm mounted on it.
    """

    @property
    def default_base(self):
        return "Go2"

    @property
    def default_arms(self):
        return {"right": "Arx5"}
    
    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.55, -0.1, 0.9),
            "empty": (-0.6, 0, 0.9),
            "table": lambda table_length: (-0.55 - table_length / 2, 0.9, 0.9),
        }

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
    
@register_robot_class("WheeledRobot")
class Kinova3Omron(Kinova3):
    """
    Variant of Panda robot with mobile base. Currently serves as placeholder class.
    """

    @property
    def default_base(self):
        return "OmronMobileBase"

    @property
    def default_arms(self):
        return {"right": "Kinova3"}
    
    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.6, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }

@register_robot_class("WheeledRobot")
class SawyerOmron(Sawyer):
    """
    Variant of Panda robot with mobile base. Currently serves as placeholder class.
    """

    @property
    def default_base(self):
        return "OmronMobileBase"

    @property
    def default_arms(self):
        return {"right": "Sawyer"}
    
    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.6, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }
    
@register_robot_class("LeggedRobot")
class GR1SchunkSVHArmsOnly(GR1ArmsOnly):
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

@register_robot_class("LeggedRobot")
class GR1SchunkSVHFixedLowerBody(GR1FixedLowerBody):
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