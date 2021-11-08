#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
 
from std_msgs.msg import String
 
def init_obstacles(cubes):
  p = geometry_msgs.msg.PoseStamped()

  for i in range(len(cubes)):
    p.pose.position.x = cubes[i].x
    p.pose.position.y = cubes[i].y
    p.pose.position.z = cubes[i].z

    if i == 0:
      scene.add_box('bucket', p, (0.2, 0.2, 0.38))
    else:
      scene.add_box('cube'+str(i-1), p, (0.05, 0.05, 0.05))

def remove_obstacles(cubes):
  for name in cubes:
    if name in scene.get_known_object_names():
      scene.remove_world_object(name)
