import numpy as np

import rospy

from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Vector3
from quadrotor_msgs.msg import RPMCommand

def tonp(obj):
  if type(obj) == list:
    return np.array([tonp(x) for x in obj])

  w = hasattr(obj, 'w')
  x = hasattr(obj, 'x')
  y = hasattr(obj, 'y')
  z = hasattr(obj, 'z')

  if w and x and y and z:
    return R.from_quat([obj.x, obj.y, obj.z, obj.w])

  elif x and y and z:
    return np.array((obj.x, obj.y, obj.z))

  assert False

def toros(obj):
  if len(obj.shape) == 2:
    if obj.shape[1] == 3:
      return [Vector3(x[0], x[1], x[2]) for x in obj]

def odomtonp(odom):
  return np.hstack((
    tonp(odom.pose.pose.position),
    tonp(odom.twist.twist.linear),
    tonp(odom.pose.pose.orientation).as_euler('ZYX')[::-1],
    tonp(odom.twist.twist.angular)))

def imutonp(imu):
  return np.hstack((
    tonp(imu.orientation).as_euler('ZYX')[::-1],
    tonp(imu.angular_velocity),
    tonp(imu.linear_acceleration)))

def rpmstoros(rpms):
    rpm_msg = RPMCommand()
    for i in range(0, len(rpms)):
        rpm_msg.motor_rpm[i] = int(rpms[i])
    return rpm_msg
