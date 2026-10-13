#!/usr/bin/env python
"""Keep the PX4 IMU and attitude streams at a fixed MAVLink rate."""

from __future__ import print_function

import collections
import threading
import time

import rospy
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu


MAV_CMD_SET_MESSAGE_INTERVAL = 511
HIGHRES_IMU = 105
ATTITUDE = 30
ATTITUDE_QUATERNION = 31


def request_rate(command, message_id, rate_hz):
    response = command(
        broadcast=False,
        command=MAV_CMD_SET_MESSAGE_INTERVAL,
        confirmation=0,
        param1=float(message_id),
        param2=1000000.0 / rate_hz,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    )
    return response.success and response.result == 0


class RateWatchdog(object):
    def __init__(self):
        self.connected_since = None
        self.raw_samples = collections.deque()
        self.data_samples = collections.deque()
        self.lock = threading.Lock()

        rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=1)
        rospy.Subscriber("/mavros/imu/data_raw", Imu,
                         self.raw_imu_cb, queue_size=400)
        rospy.Subscriber("/mavros/imu/data", Imu,
                         self.imu_cb, queue_size=400)

    def state_cb(self, message):
        if message.connected:
            if self.connected_since is None:
                self.connected_since = time.time()
        else:
            self.connected_since = None
            with self.lock:
                self.raw_samples.clear()
                self.data_samples.clear()

    def raw_imu_cb(self, _message):
        with self.lock:
            self.record(self.raw_samples)

    def imu_cb(self, _message):
        with self.lock:
            self.record(self.data_samples)

    @staticmethod
    def record(samples):
        now = time.time()
        samples.append(now)
        while samples and samples[0] < now - 4.0:
            samples.popleft()

    def rate(self, samples, window=3.0):
        now = time.time()
        with self.lock:
            recent = [stamp for stamp in samples if stamp >= now - window]
        if len(recent) < 2 or recent[-1] == recent[0]:
            return 0.0
        return float(len(recent) - 1) / (recent[-1] - recent[0])

    def connected_for(self):
        if self.connected_since is None:
            return 0.0
        return time.time() - self.connected_since


def set_rates(command, rate_hz):
    results = [
        request_rate(command, HIGHRES_IMU, rate_hz),
        request_rate(command, ATTITUDE_QUATERNION, rate_hz),
        request_rate(command, ATTITUDE, rate_hz),
    ]
    return all(results)


def main():
    rospy.init_node("px4_imu_rate_requester")
    rate_hz = float(rospy.get_param("~rate_hz", 200.0))
    min_rate_hz = float(rospy.get_param("~min_rate_hz", rate_hz * 0.85))
    stable_seconds = float(rospy.get_param("~stable_seconds", 5.0))
    check_period = float(rospy.get_param("~check_period", 5.0))
    service_name = "/mavros/cmd/command"
    watchdog = RateWatchdog()

    rospy.loginfo("Waiting for %s before requesting %.0f Hz PX4 IMU streams",
                  service_name, rate_hz)
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as error:
        rospy.logerr("MAVROS command service is unavailable: %s", error)
        return

    command = rospy.ServiceProxy(service_name, CommandLong, persistent=True)
    configured = False
    while not rospy.is_shutdown():
        if watchdog.connected_for() < stable_seconds:
            configured = False
            rospy.sleep(0.5)
            continue

        raw_rate = watchdog.rate(watchdog.raw_samples)
        data_rate = watchdog.rate(watchdog.data_samples)
        needs_update = (not configured or raw_rate < min_rate_hz or
                        data_rate < min_rate_hz)
        if not needs_update:
            rospy.sleep(check_period)
            continue

        try:
            if set_rates(command, rate_hz):
                if configured:
                    rospy.logwarn("IMU rate low (raw %.1f Hz, data %.1f Hz); "
                                  "reapplying %.0f Hz", raw_rate, data_rate,
                                  rate_hz)
                else:
                    rospy.loginfo("PX4 IMU streams requested at %.0f Hz",
                                  rate_hz)
                configured = True
            else:
                rospy.logwarn("PX4 rejected an IMU stream-rate request")
        except rospy.ServiceException as error:
            rospy.logwarn("PX4 stream-rate request failed: %s", error)
            command = rospy.ServiceProxy(service_name, CommandLong,
                                         persistent=True)
            configured = False
        rospy.sleep(check_period)


if __name__ == "__main__":
    main()
