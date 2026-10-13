#!/usr/bin/env python
from __future__ import print_function

import argparse
import math
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


imu_samples = []
vins_samples = []


def imu_callback(msg):
    imu_samples.append((
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z,
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z,
    ))


def vins_callback(msg):
    q = msg.pose.pose.orientation
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch_arg = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_arg)))
    vins_samples.append((roll, pitch))


def mean(values):
    return sum(values) / float(len(values))


def angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def log_line(message, log_path):
    line = "%s %s" % (time.strftime("%Y-%m-%d %H:%M:%S"), message)
    print(line)
    try:
        with open(log_path, "a") as stream:
            stream.write(line + "\n")
    except IOError:
        pass


def collect_and_check(args):
    del imu_samples[:]
    del vins_samples[:]
    deadline = time.time() + args.timeout
    while not rospy.is_shutdown() and time.time() < deadline:
        if imu_samples and vins_samples:
            break
        time.sleep(0.05)
    if not imu_samples or not vins_samples:
        return False, "FAIL missing IMU or VINS data"

    del imu_samples[:]
    del vins_samples[:]
    end_time = time.time() + args.duration
    while not rospy.is_shutdown() and time.time() < end_time:
        time.sleep(0.02)

    if len(imu_samples) < 50 or len(vins_samples) < 50:
        return False, "FAIL insufficient samples imu=%d vins=%d" % (
            len(imu_samples), len(vins_samples))

    ax = mean([sample[0] for sample in imu_samples])
    ay = mean([sample[1] for sample in imu_samples])
    az = mean([sample[2] for sample in imu_samples])
    raw_roll = math.atan2(ay, az)
    raw_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    vins_roll = mean([sample[0] for sample in vins_samples])
    vins_pitch = mean([sample[1] for sample in vins_samples])

    accel_norms = [math.sqrt(s[0] ** 2 + s[1] ** 2 + s[2] ** 2)
                   for s in imu_samples]
    accel_mean = mean(accel_norms)
    accel_std = math.sqrt(mean([(value - accel_mean) ** 2
                                for value in accel_norms]))
    gyro_rms = math.sqrt(mean([s[3] ** 2 + s[4] ** 2 + s[5] ** 2
                               for s in imu_samples]))

    roll_error = abs(angle_error(vins_roll, raw_roll))
    pitch_error = abs(angle_error(vins_pitch, raw_pitch))
    deg = 180.0 / math.pi
    stable = (gyro_rms <= args.max_gyro_rms and
              accel_std <= args.max_accel_std and
              abs(accel_mean - args.gravity) <= args.max_gravity_error)
    matched = max(roll_error, pitch_error) * deg <= args.threshold_deg
    passed = stable and matched
    status = "PASS" if passed else "FAIL"
    result = ("%s raw_rp=(%.2f,%.2f) vins_rp=(%.2f,%.2f) "
              "err=(%.2f,%.2f) gyro_rms=%.4f accel=%.4f+/-%.4f "
              "samples=(%d,%d)") % (
                  status, raw_roll * deg, raw_pitch * deg,
                  vins_roll * deg, vins_pitch * deg,
                  roll_error * deg, pitch_error * deg,
                  gyro_rms, accel_mean, accel_std,
                  len(imu_samples), len(vins_samples))
    return passed, result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--settle", type=float, default=8.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--threshold-deg", type=float, default=2.0)
    parser.add_argument("--max-resets", type=int, default=3)
    parser.add_argument("--max-gyro-rms", type=float, default=0.05)
    parser.add_argument("--max-accel-std", type=float, default=0.15)
    parser.add_argument("--max-gravity-error", type=float, default=0.5)
    parser.add_argument("--gravity", type=float, default=9.805)
    parser.add_argument("--log", default="/tmp/vins_startup_check.log")
    args = parser.parse_args()

    rospy.init_node("vins_gravity_check", anonymous=True)
    rospy.Subscriber("/mavros/imu/data_raw", Imu, imu_callback, queue_size=400)
    rospy.Subscriber("/uav1/vins_estimator/imu_propagate", Odometry,
                     vins_callback, queue_size=400)
    restart_pub = rospy.Publisher("/uav1/vins_restart", Bool, queue_size=1)

    for attempt in range(args.max_resets + 1):
        time.sleep(args.settle)
        passed, result = collect_and_check(args)
        log_line("attempt=%d/%d %s" %
                 (attempt + 1, args.max_resets + 1, result), args.log)
        if passed:
            return 0
        if attempt < args.max_resets:
            wait_end = time.time() + 2.0
            while restart_pub.get_num_connections() == 0 and time.time() < wait_end:
                time.sleep(0.05)
            restart_pub.publish(Bool(data=True))
            log_line("restart=%d/%d published" %
                     (attempt + 1, args.max_resets), args.log)

    log_line("BLOCK px4ctrl was not started", args.log)
    return 1


if __name__ == "__main__":
    sys.exit(main())
