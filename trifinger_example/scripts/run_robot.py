import argparse
from copy import deepcopy

import numpy as np

import trifinger_simulation
import trifinger_simulation.tasks.move_cube as task
from trifinger_example.robot import Robot
import trifinger_example.utils as utils


# example marker (only used in when running in simulation)
example_marker = None
goal_marker = None
# example coordinates in the frame of the object
x_local_lst = [
    #[-0.5*task._CUBE_WIDTH - 0.01, 0., 0.],
    #[0, 0., 0.5*task._CUBE_WIDTH + 0.01],
    [0, 0.5*task._CUBE_WIDTH + 0.01, 0 ],
    [ 0.5*task._CUBE_WIDTH + 0.01, 0., 0.],
]
arm1 = None
arm2 = None
move = None



def get_action(robot, t):
    """Determine the robot actions."""

    # get observations
    robot_observation, object_observation = robot.get_observation(t)
    goal = robot.goal.position

    # forward kinematics
    tip_positions = robot.kinematics.forward_kinematics(robot_observation.position)

    # transform from object to world space
    # x_global_lst = [utils.to_world_space(x_local, object_observation) for x_local in x_local_lst]
    x_global_lst = [object_observation.position + np.asarray([0, 0., 0.5*task._CUBE_WIDTH + 0.01]),
                    np.asarray([0.18, 0.18, 0.08])]

    # update position of example marker based on transformed points
    if goal_marker is not None:
        goal_marker.set_state([goal])

    # global arm1
    # global arm2
    # if arm1 is None:
    #     dist_1 = np.linalg.norm(tip_positions - x_global_lst[0],axis=0)
    #     dist_2 = np.linalg.norm(tip_positions - x_global_lst[1],axis=0)
    #     print(dist_1, dist_2)
    #     arm1=np.argmin(dist_1)
    #     arm2=np.argmin(dist_2)
    #     print(arm1,arm2)
    # do inverse kinematics
    global move
    object_pos  = object_observation.position
    diff = object_pos - tip_positions

    target = tip_positions.copy()
    target[0] = object_observation.position - (diff[0]/np.linalg.norm(diff[0])*0.5*task._CUBE_WIDTH)
    target[1] = object_observation.position - (diff[1]/np.linalg.norm(diff[1])*0.5*task._CUBE_WIDTH)
    target[2] = object_observation.position - (diff[2]/np.linalg.norm(diff[2])*0.5*task._CUBE_WIDTH)

    if(np.linalg.norm(tip_positions[0] - target[0])<0.02):
        move = 1

    # if(np.linalg.norm(tip_positions[0] - x_global_lst[0])>0.02):
    #     move = None
    if move is not None:
        target[0] = target[0] + 0.3*(goal-object_pos)
        target[1] = target[1] + 0.3*(goal-object_pos)
        target[2] = target[2] + 0.3*(goal-object_pos)
        print(target[0])
    if example_marker is not None:
         example_marker.set_state(target)


    target_position, _ = robot.kinematics.inverse_kinematics(
        tip_target_positions=target,
        joint_angles_guess=robot_observation.position,
        max_iterations = 100
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
                            number_of_goals=3,
                            goal_size=0.005,
                            initial_position=[0.18, 0.18, 0.08],
                        )
        global goal_marker # please accept my apologies
        goal_marker = trifinger_simulation.visual_objects.Marker(
                            number_of_goals=1,
                            goal_size=0.008,
                            initial_position=[0.18, 0.18, 0.08],
                        )

    # control loop
    while t < args.episode_length:
        # get action
        target_position, torque = get_action(robot, t)
        # send action to robot
        t = robot.append_desired_action(
            position=target_position,
            torque=torque
        )

if __name__ == "__main__":
    main()
