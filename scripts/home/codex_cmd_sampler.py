#!/usr/bin/env python2
import math

import rospy
from mavros_msgs.msg import AttitudeTarget
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt8


state = {'fsm': -1, 'imu': None, 'cmd': None, 'att': None}


def fsm_callback(message):
    state['fsm'] = message.data


def imu_callback(message):
    state['imu'] = message


def command_callback(message):
    if state['fsm'] == 2:
        state['cmd'] = message


def attitude_callback(message):
    if state['fsm'] == 2:
        state['att'] = message


rospy.init_node('codex_cmd_sampler', anonymous=True)
rospy.Subscriber('/fsm_state', UInt8, fsm_callback)
rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
rospy.Subscriber('/position_cmd', PositionCommand, command_callback)
rospy.Subscriber('/mavros/setpoint_raw/attitude', AttitudeTarget, attitude_callback)

end = rospy.Time.now() + rospy.Duration(7.0)
rate = rospy.Rate(100)
while not rospy.is_shutdown() and rospy.Time.now() < end:
    rate.sleep()

print 'FSM_LAST', state['fsm']
if state['cmd']:
    command = state['cmd']
    print 'CMD p=(%.3f,%.3f,%.3f) v=(%.3f,%.3f,%.3f) a=(%.3f,%.3f,%.3f) yaw=%.3f' % (
        command.position.x, command.position.y, command.position.z,
        command.velocity.x, command.velocity.y, command.velocity.z,
        command.acceleration.x, command.acceleration.y, command.acceleration.z,
        command.yaw)
else:
    print 'CMD none while CMD_CTRL'

if state['att'] and state['imu']:
    target = state['att'].orientation
    current = state['imu'].orientation
    x = current.w * target.x - current.x * target.w - current.y * target.z + current.z * target.y
    y = current.w * target.y + current.x * target.z - current.y * target.w - current.z * target.x
    z = current.w * target.z - current.x * target.y + current.y * target.x + current.z * target.w
    w = current.w * target.w + current.x * target.x + current.y * target.y + current.z * target.z
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    print 'ATT q=(%.6f,%.6f,%.6f,%.6f) thrust=%.4f REL_RPY_DEG=(%.2f,%.2f,%.2f)' % (
        target.x, target.y, target.z, target.w, state['att'].thrust, roll, pitch, yaw)
else:
    print 'ATT or IMU none while CMD_CTRL'
