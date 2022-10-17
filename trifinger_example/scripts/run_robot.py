import argparse
from copy import deepcopy

import numpy as np

import trifinger_simulation
import trifinger_simulation.tasks.move_cube as task
from trifinger_example.robot import Robot
import trifinger_example.utils as utils


# example marker (only used in when running in simulation)
example_marker = None
# example coordinates in the frame of the object
x_local_lst = [
    [-0.5*task._CUBE_WIDTH - 0.01, 0., 0.],
    [ 0.5*task._CUBE_WIDTH + 0.01, 0., 0.],
]


def get_action(robot, t):
    """Determine the robot actions."""

    # get observations
    robot_observation, object_observation = robot.get_observation(t)

    # forward kinematics
    tip_positions = robot.kinematics.forward_kinematics(robot_observation.position)

    # transform from object to world space
    x_global_lst = [utils.to_world_space(x_local, object_observation) for x_local in x_local_lst]

    # update position of example marker based on transformed points
    if example_marker is not None:
        example_marker.set_state(x_global_lst)

    # do inverse kinematics
    tip_target_positions = tip_positions
    target_position, _ = robot.kinematics.inverse_kinematics(
        tip_target_positions=tip_target_positions,
        joint_angles_guess=robot_observation.position
    )

    # return target position and torque, can also be combined
    return target_position.tolist(), None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Run in simulation instead of on real robot."
    )
    parser.add_argument(
        "--episode_length",
        type=int,
        default=10000,
        help="Episode length in ms (robot steps)."
    )
    args = parser.parse_args()

    # initialize robot platform
    robot = Robot(
        simulate=args.simulate,
        episode_length=args.episode_length
    )
    t = 0

    # can use markers to visualize positions (when running in simulation)
    if args.simulate:
        global example_marker # please accept my apologies
        example_marker = trifinger_simulation.visual_objects.Marker(
                            number_of_goals=2,
                            goal_size=0.005,
                            initial_position=[0.18, 0.18, 0.08],
                        )

    # control loop
    while t <= args.episode_length:
        # get action
        target_position, torque = get_action(robot, t)
        # send action to robot
        t = robot.append_desired_action(
            position=target_position,
            torque=torque
        )

if __name__ == "__main__":
    main()