#!/usr/bin/env python
"""Request the PX4 IMU and attitude streams at a fixed MAVLink rate."""

from __future__ import print_function

import rospy
from mavros_msgs.srv import CommandLong


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


def main():
    rospy.init_node("px4_imu_rate_requester")
    rate_hz = float(rospy.get_param("~rate_hz", 200.0))
    retries = int(rospy.get_param("~retries", 60))
    service_name = "/mavros/cmd/command"

    rospy.loginfo("Waiting for %s before requesting %.0f Hz PX4 IMU streams",
                  service_name, rate_hz)
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as error:
        rospy.logerr("MAVROS command service is unavailable: %s", error)
        return

    command = rospy.ServiceProxy(service_name, CommandLong)
    for _ in range(retries):
        try:
            imu_ok = request_rate(command, HIGHRES_IMU, rate_hz)
            attitude_ok = request_rate(command, ATTITUDE_QUATERNION, rate_hz)
            attitude_legacy_ok = request_rate(command, ATTITUDE, rate_hz)
            if imu_ok and attitude_ok and attitude_legacy_ok:
                rospy.loginfo("PX4 HIGHRES_IMU, ATTITUDE and ATTITUDE_QUATERNION set to %.0f Hz",
                              rate_hz)
                return
            rospy.logwarn("PX4 rejected the IMU stream-rate request; retrying")
        except rospy.ServiceException as error:
            rospy.logwarn("PX4 stream-rate request failed: %s", error)
        rospy.sleep(1.0)

    rospy.logerr("Could not configure PX4 IMU streams after %d attempts", retries)


if __name__ == "__main__":
    main()
