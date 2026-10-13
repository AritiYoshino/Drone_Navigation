#!/usr/bin/env python

from __future__ import print_function

import argparse
import math

import rospy
from geometry_msgs.msg import PoseStamped


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactively publish EGO-Planner goal points."
    )
    parser.add_argument(
        "--topic", default="/move_base_simple/goal", help="goal topic"
    )
    parser.add_argument("--frame-id", default="world", help="goal frame")
    parser.add_argument(
        "--repeat", type=int, default=3, help="publish count for each goal"
    )
    return parser.parse_args(rospy.myargv()[1:])


def parse_goal(line):
    fields = line.replace(",", " ").split()
    if len(fields) != 3:
        raise ValueError("enter exactly three values: x y z")

    goal = tuple(float(value) for value in fields)
    if any(math.isnan(value) or math.isinf(value) for value in goal):
        raise ValueError("coordinates must be finite numbers")
    if goal[2] < -0.1:
        raise ValueError("EGO-Planner ignores goals with z < -0.1")
    return goal


def publish_goal(publisher, frame_id, goal, repeat):
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.pose.position.x = goal[0]
    message.pose.position.y = goal[1]
    message.pose.position.z = goal[2]
    message.pose.orientation.w = 1.0

    if publisher.get_num_connections() == 0:
        rospy.logwarn("No subscriber on %s; goal may not be received.", publisher.resolved_name)

    for _ in range(repeat):
        message.header.stamp = rospy.Time.now()
        publisher.publish(message)
        rospy.sleep(0.05)

    rospy.loginfo("Goal sent: x=%.3f y=%.3f z=%.3f", *goal)


def main():
    args = parse_args()
    if args.repeat < 1:
        raise SystemExit("--repeat must be at least 1")

    rospy.init_node("realtime_ego_goal", anonymous=True)
    publisher = rospy.Publisher(args.topic, PoseStamped, queue_size=1)
    rospy.sleep(0.5)

    try:
        input_func = raw_input
    except NameError:
        input_func = input

    print("Enter an absolute goal as: x y z")
    print("Enter q to quit.")

    while not rospy.is_shutdown():
        try:
            line = input_func("goal> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("")
            break

        if not line:
            continue
        if line.lower() in ("q", "quit", "exit"):
            break

        try:
            goal = parse_goal(line)
        except ValueError as error:
            print("Invalid goal: {0}".format(error))
            continue

        publish_goal(publisher, args.frame_id, goal, args.repeat)


if __name__ == "__main__":
    main()
