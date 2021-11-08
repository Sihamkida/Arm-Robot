#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil
 
currentJointState = JointState()
def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg

def __execute__(tmp):
  pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
 
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  print '   Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    print '   Published!'
    rate.sleep()
  rospy.sleep(2)
  print 'end!'


def close():
  print('Closing gripper')
  __execute__(0.7)

def open():
  print('Opening gripper')
  __execute__(0.005)
