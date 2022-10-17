import json
import os
from time import time, sleep

import numpy as np

import trifinger_simulation
from trifinger_simulation import trifingerpro_limits
import trifinger_simulation.tasks.move_cube as task
import trifinger_simulation.visual_objects
from trifinger_simulation.pinocchio_utils import Kinematics

from .utils import random_yaw_orientation


class Robot:
    """Implements interactions with robot (simulated or real)."""

    def __init__(self, simulate, episode_length=10000):
        self.simulate = simulate
        self.episode_length = episode_length

        # sample random goal pose for the cube
        self.goal = task.sample_goal(difficulty=1)
        if not simulate:
            self._save_goal()

        # get robot platform object (simulated or real)
        if simulate:
            self.platform = self._get_platform_sim(self.goal)
        else:
            self.platform = self._get_platform_real()

        self.kinematics = self._get_kinematics()
        self.time_of_last_step = None

        # already do one step here to be able to get observation
        self.append_desired_action(
            position=trifingerpro_limits.robot_position.default
        )


    def _save_goal(self):
        """Save goal to file for visualization purposes."""

        goals_path = "/output/goals.json"
        goals_dict = {
            "difficulty": 1,
            "goal": [{
                    "position": self.goal.position.tolist(),
                    "orientation": self.goal.orientation.tolist(),
                    "t_start": 0,
                    "t_end": self.episode_length
            }]

        }
        with open(goals_path, "w") as f:
            json.dump(goals_dict, f, indent=4)

    def _get_platform_sim(self, goal):
        """Initialize simulation."""
        
        initial_robot_position = trifingerpro_limits.robot_position.default

        # initial position of cube in the middle of the arena
        # with random orientation
        position = np.array((0., 0., task._CUBE_WIDTH / 2))
        orientation = random_yaw_orientation()
        initial_object_pose = task.Pose(
            position=position,
            orientation=orientation
        )

        platform = trifinger_simulation.TriFingerPlatform(
            visualization=True,
            initial_robot_position=initial_robot_position,
            initial_object_pose=initial_object_pose,
        )

        # visualize the goal
        trifinger_simulation.visual_objects.CubeMarker(
            width=task._CUBE_WIDTH,
            position=goal.position,
            orientation=goal.orientation,
            pybullet_client_id=platform.simfinger._pybullet_client_id,
        )

        return platform

    def _get_platform_real(self):
        """Initialize real robot platform."""

        # import here to avoid error when running locally in simulation
        import robot_fingers
        platform = robot_fingers.TriFingerPlatformWithObjectFrontend()
        return platform

    def _get_kinematics(self):
        """Get kinematics object."""

        robot_properties_path = os.path.join(
            os.path.dirname(trifinger_simulation.__file__), "robot_properties_fingers"
        )
        tip_link_names = [
            "finger_tip_link_0",
            "finger_tip_link_120",
            "finger_tip_link_240",
        ]
        urdf_file = trifinger_simulation.finger_types_data.get_finger_urdf("trifingerpro")
        finger_urdf_path = os.path.join(
            robot_properties_path, "urdf", urdf_file
        )
        return Kinematics(
            finger_urdf_path, tip_link_names
        )

    def append_desired_action(self, torque=None, position=None):
        """Append action to queue."""

        kwargs = {}
        if position is not None:
            kwargs["position"] = position
        if torque is not None:
            kwargs["torque"] = torque
        robot_action = self.platform.Action(**kwargs)
        t = self.platform.append_desired_action(robot_action)

        # avoid simulator running faster than real time
        if self.simulate:
            if self.time_of_last_step is not None:
                sleep(max(0.001 - (time() - self.time_of_last_step), 0.))
            self.time_of_last_step = time()

        return t

    def get_observation(self, t):
        """Get robot and object observation at time step t."""

        robot_observation = self.platform.get_robot_observation(t)
        camera_observation = self.platform.get_camera_observation(t)
        object_observation = camera_observation.object_pose
        return robot_observation, object_observation