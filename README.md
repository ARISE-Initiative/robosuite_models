# robosuite_models

A collection of robot models tailored to robosuite. 

This repository also provide examples of how to create a custom composable robot model with [robosuite](https://github.com/ARISE-Initiative/robosuite).

See [compositional.py](robosuite_models/robots/compositional.py): how to compose a new robot model.

| Robot Model | Names |
| --- | --- |
| `RobotBaseModel` | aloha_mount, b1_base, go2_base |
| `GripperModel` | ability_hands, aloha_gripper, g1_three_finger_gripper, pr2_gripper, schunk_svh_hands, umi_gripper, yumi_gripper, z1_gripper |
| `ManipulatorModel` | aloha_robot, arx5_robot, g1_robot, h1_robot, pr2_robot, vx330s_robot, yumi_robot, z1_robot |

Example code:

```py
from robosuite.robots import register_robot_class
from robosuite.models.robots import Panda
import robosuite as suite
from robosuite.controllers import load_composite_controller_config
import mujoco


@register_robot_class("WheeledRobot")
class MobilePanda(Panda):
    @property
    def default_base(self):
        return "OmronMobileBase"

    @property
    def default_arms(self):
        return {"right": "Panda"}

# Create environment
env = suite.make(
    env_name="Lift",
    robots="MobilePanda",
    controller_configs=load_composite_controller_config(controller="BASIC"),
    has_renderer=True,
    has_offscreen_renderer=False,
    render_camera="agentview",
    use_camera_obs=False,
    control_freq=20,
)

# Run the simulation, and visualize it
env.reset()
mujoco.viewer.launch(env.sim.model._model, env.sim.data._data)
```

## Example

To load a robot model to see what it looks like, you can use the following command:
```sh
python examples/load_compositional_robot_example.py --robots UR5eOmron  --controller BASIC
```

## Contributing

Run `pre-commit run --all-files` before open a pull request.
