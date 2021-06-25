#!/usr/bin/env python3

import rospy
import argparse
from tbd_baxter_tools.motion import move_to_posture


def main():
    # start the rosnode
    rospy.init_node("move_to_baxter_posture_node")
    # parse the arguments
    parser = argparse.ArgumentParser(description="Scripts that move the robot to the given posture")
    parser.add_argument("posture_name", help="Name of the posture", type=str)
    parser.add_argument("--record_path", help="Path to the file that store the postures", type=str)
    args = parser.parse_args()
    # parse the arguments

    rospy.loginfo(f"Moving to posture:{args.posture_name}")
    if args.record_path:
        move_to_posture(args.posture_name, record_path=args.record_path)
    else:
        move_to_posture(args.posture_name)


if __name__ == "__main__":
    main()
