#!/usr/bin/env python3
"""Play back TriCameraObjectObservations from a log file.

This is an extended version of the original script of the
trifinger_object_tracking package that also reads the robot log to get the time
step information needed to visualise the changing goal position.
"""
import argparse
import json
import pathlib
import sys

import cv2
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import robot_fingers
import trifinger_object_tracking.py_object_tracker
import trifinger_object_tracking.py_tricamera_types as tricamera
from trifinger_cameras import utils
import quaternion


CAMERA_NAMES = ["camera60", "camera180", "camera300"]

def get_pose_from_keypoints(keypoints, dimensions=(0.065, 0.065, 0.065)):
    center = np.mean(keypoints, axis=0)
    kp_centered = np.array(keypoints) - center
    kp_scaled = kp_centered/np.array(dimensions)*2.

    loc_kps = []
    for i in range(3):
        # convert to binary representation
        str_kp = "{:03b}".format(i)
        # set components of keypoints according to digits in binary representation
        loc_kp = [(1. if str_kp[i] == "0" else -1.) for i in range(3)][::-1]
        loc_kps.append(loc_kp)
    K_loc = np.transpose(np.array(loc_kps))
    K_loc_inv = np.linalg.inv(K_loc)
    K_glob = np.transpose(kp_scaled[0:3])
    R = np.matmul(K_glob, K_loc_inv)
    quat = quaternion.from_rotation_matrix(R)

    return center, np.array([quat.x, quat.y, quat.z, quat.w])


def blend_images(images1, images2, alpha=0.7):
    res = []
    for image1, image2 in zip(images1, images2):
        blended = alpha*image1  + (1. - alpha)*image2
        res.append(np.uint8(blended))
    return res


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "robot_log",
        type=pathlib.Path,
        help="Path to the robot log file.",
    )
    parser.add_argument(
        "camera_log",
        type=pathlib.Path,
        help="Path to the camera log file.",
    )
    parser.add_argument(
        "--visualize-object-pose",
        "-v",
        action="store_true",
        help="""Visualize detected object pose.  This expects files
        camera{60,180,300}.yml with calibration parameters to exist in the same
        directory as the given camera log file.
        """,
    )
    parser.add_argument(
        "--visualize-goal-pose",
        "-g",
        type=pathlib.Path,
        metavar="GOAL_FILE",
        help="Visualize goal from the specified JSON file.",
    )
    parser.add_argument(
        "--goal-as-circle",
        action="store_true",
        help="Visualize goal with a circle instead of a cube.",
    )
    parser.add_argument(
        "--object",
        type=str,
        default="cube_v2",
        help="Name of the object model used for visualisation.  Default: %(default)s.",
    )
    parser.add_argument(
        "--show-confidence",
        action="store_true",
        help="Print the object pose confidence in the images.",
    )
    parser.add_argument(
        "--unfiltered",
        action="store_true",
        help="Use the unfiltered object pose.",
    )
    parser.add_argument(
        "--plot-cube-position",
        "-p",
        action="store_true",
        help="Plot cube position",
    )
    parser.add_argument(
        "--save-video",
        type=str,
        metavar="VIDEO_FILE",
        help="""Save the images of the camera selected by --camera to a AVI
        video file.  Expects as argument the output path.
        """,
    )
    parser.add_argument(
        "--camera",
        "-c",
        choices=CAMERA_NAMES,
        help="Name of the camera.  Used by --save-video.",
    )
    parser.add_argument(
        "--show_reset",
        "-r",
        action="store_true",
        help="Show footage between episodes captured during cube reset.",
    )
    args = parser.parse_args()

    if not args.camera_log.exists():
        print("{} does not exist.".format(args.camera_log))
        sys.exit(1)
    if not args.robot_log.exists():
        print("{} does not exist.".format(args.robot_log))
        sys.exit(1)

    if args.visualize_goal_pose:
        if not args.visualize_goal_pose.exists():
            print("{} does not exist.".format(args.visualize_goal_pose))
            sys.exit(1)

        with open(args.visualize_goal_pose, "r") as fh:
            goal_dict = json.load(fh)

        goal = goal_dict["goal"]

        goal_pose = trifinger_object_tracking.py_object_tracker.ObjectPose()
        current_goal = None

    calib_files = []
    if args.visualize_object_pose or args.visualize_goal_pose:
        for name in CAMERA_NAMES:
            calib_file = args.camera_log.parent / (name + ".yml")
            if calib_file.exists():
                calib_files.append(str(calib_file))
            else:
                print("{} does not exist.".format(calib_file))
                sys.exit(1)
        model = trifinger_object_tracking.py_object_tracker.get_model_by_name(
            args.object
        )
        cube_visualizer = tricamera.CubeVisualizer(model, calib_files)

    log = robot_fingers.TriFingerPlatformWithObjectLog(
        str(args.robot_log), str(args.camera_log)
    )

    # assume camera rate of 10 Hz
    fps = 10
    interval = 100

    t0 = log.get_first_timeindex()

    if args.save_video:
        if not args.camera:
            print("--camera is required for saving video.")
            sys.exit(1)

        camera_index = CAMERA_NAMES.index(args.camera)
        first_img = utils.convert_image(
            log.get_camera_observation(t0).cameras[0].image
        )
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        video_writer = cv2.VideoWriter(
            args.save_video, fourcc, fps, first_img.shape[:2]
        )

    for t in range(
        log.get_first_timeindex(), log.get_last_timeindex() + 1, interval
    ):
        observation = log.get_camera_observation(t)
        images = [
            utils.convert_image(camera.image) for camera in observation.cameras
        ]

        if args.unfiltered:
            object_pose = observation.object_pose
        else:
            object_pose = observation.filtered_object_pose

        if args.visualize_goal_pose:
            if goal and t > goal[0]["t_start"]:
                current_goal = goal.pop(0)
                if "position" in current_goal:
                    goal_pose.position = current_goal["position"]
                    goal_pose.orientation = current_goal.get("orientation", (0, 0, 0, 1))
                else:
                    position, orientation = get_pose_from_keypoints(current_goal["keypoints"])
                    goal_pose.position = position
                    goal_pose.orientation = orientation
            if current_goal is not None and t < current_goal["t_end"]:
                if args.goal_as_circle:
                    images = cube_visualizer.draw_circle(images, goal_pose, True, opacity=0.6, scale=0.64)
                else:
                    images = cube_visualizer.draw_cube(images, goal_pose, True, opacity=0.7)

        if args.visualize_object_pose:
            images = cube_visualizer.draw_cube(images, object_pose, False)

        if args.show_confidence:
            images = [
                cv2.putText(
                    image,
                    "confidence: %.2f" % object_pose.confidence,
                    (0, image.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                )
                for image in images
            ]

        # by default, only show episodes and not resets
        if (current_goal is not None and t < current_goal["t_end"]) or args.show_reset:
            if args.save_video:
                video_writer.write(images[camera_index])
            else:
                for i, name in enumerate(CAMERA_NAMES):
                    cv2.imshow(name, images[i])

                # stop if either "q" or ESC is pressed
                if cv2.waitKey(interval) in [ord("q"), 27]:  # 27 = ESC
                    break

            if args.plot_cube_position:
                plt.scatter(
                    observation.cameras[0].timestamp,
                    object_pose.position[0],
                    color="red",
                )
                plt.scatter(
                    observation.cameras[0].timestamp,
                    object_pose.position[1],
                    color="green",
                )
                plt.scatter(
                    observation.cameras[0].timestamp,
                    object_pose.position[2],
                    color="blue",
                )

                plt.title("Cube Position")
                legend_elements = [
                    Line2D(
                        [0],
                        [0],
                        marker="o",
                        color="w",
                        label="x",
                        markerfacecolor="r",
                    ),
                    Line2D(
                        [0],
                        [0],
                        marker="o",
                        color="w",
                        label="y",
                        markerfacecolor="g",
                    ),
                    Line2D(
                        [0],
                        [0],
                        marker="o",
                        color="w",
                        label="z",
                        markerfacecolor="b",
                    ),
                ]
                plt.legend(handles=legend_elements, loc="upper right")

                plt.pause(0.01)


if __name__ == "__main__":
    main()
