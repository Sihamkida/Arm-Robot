#!/usr/bin/env python

import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
from std_msgs.msg import String
from gazebo_msgs import msg
from sensor_msgs.msg import JointState
import math

class position:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ox = 0.0
        self.oy = 0.0
        self.oz = 0.0
        self.ow = 0.0

#Create callback. This is what happens when a new message is received
def sub_cal(msg):
    n = len(msg.name)-2
    global objects
    objects = [position() for i in range(n)] 
    for i in range(n-1,-1,-1):
        objects[n-1-i].x = msg.pose[i+2].position.x
        objects[n-1-i].y = msg.pose[i+2].position.y
        objects[n-1-i].z = msg.pose[i+2].position.z
        objects[n-1-i].ox = msg.pose[i+2].orientation.x
        objects[n-1-i].oy = msg.pose[i+2].orientation.y
        objects[n-1-i].oz = msg.pose[i+2].orientation.z
        objects[n-1-i].ow = msg.pose[i+2].orientation.w

#Initialize publisher
def initSubscriber():
    rospy.Subscriber('gazebo/model_states', msg.ModelStates, sub_cal, queue_size=1000)

def get_positions():
    print("get object positions")
    n = 1
    global objects
    objects = 0
    while(n == 1):
        rospy.sleep(0.1)
        n = len(objects)
        print('Number of objects: {}'.format(n))
        if(n > 0):
            print('bucket\nx: {}\ny: {}\nz: {}'.format(objects[0].x, objects[0].y, objects[0].z))
            for i in range(1,n):
                    print('cube{}\nx: {}\ny: {}\nz: {}'.format(n-1-i, objects[i].x, objects[i].y, objects[i].z))
    return objects

def init_obstacles(cubes):
  p = geometry_msgs.msg.PoseStamped()
  p.header.frame_id = robot.get_planning_frame()

  for i in range(len(cubes)):
    print i
    p.pose.position.x = cubes[i].x
    p.pose.position.y = cubes[i].y
    p.pose.position.z = cubes[i].z
    p.pose.orientation.x = cubes[i].ox
    p.pose.orientation.y = cubes[i].oy
    p.pose.orientation.z = cubes[i].oz
    p.pose.orientation.w = cubes[i].ow

    if i == 0:
      scene.add_box('bucket', p, (0.2, 0.2, 0.38))
      print 'Bucket on its way'
    else:
      scene.add_box('cube'+str(i-1), p, (0.05, 0.05, 0.05))
      print 'Box on its way'

def remove_obstacles(cubes):
  for name in cubes:
    if name in scene.get_known_object_names():
      scene.remove_world_object(name)

def init_movement_commander():
  ## BEGIN_TUTORIAL
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  # rospy.init_node('move_group_python_interface_tutorial',
                  # anonymous=True)
 
  global robot
  global scene
  global group
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")
 
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
 
  print "============ Starting tutorial "
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()
  ## We can also print the name of the end-effector link for this group
  print "============ End effector frame: %s" % group.get_end_effector_link()
  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
 
  ## Let's setup the planner
  group.set_planning_time(2.0)
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)
  group.set_max_velocity_scaling_factor(1.0)
  group.set_max_acceleration_scaling_factor(1.0)

def move_to_object_linear(test_object, offset):
  print "============ Generating linear plan"
  pose_goal = group.get_current_pose().pose
  waypoints1 = []
  waypoints1.append(pose_goal)
  pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
  pose_goal.position.x = test_object.x
  pose_goal.position.y = test_object.y
  pose_goal.position.z = test_object.z + offset
  waypoints1.append(pose_goal)

  fraction = 0
  attempts = 0
  while (fraction <= 0.9 and attempts <= 30):
    (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints1,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
    plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
    attempts = attempts + 1
  print fraction
  if fraction < 0.5:
    move_to_object(test_object, offset)
  else:
    group.execute(plan1,wait=True)

def move_to_object(test_object, offset):
  print "============ Generating alternative plan"
  pose_goal = group.get_current_pose().pose
  pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
  pose_goal.position.x = test_object.x
  pose_goal.position.y = test_object.y
  pose_goal.position.z = test_object.z + offset
  group.set_pose_target(pose_goal)
  plan1 = group.plan()
  group.go(wait=True)
  print "moving"

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

currentJointState = JointState()
rospy.init_node('main_control_node')
init_movement_commander()
initSubscriber()
objects = get_positions()
remove_obstacles(objects)
init_obstacles(objects)
open()
n = len(objects) - 1
for i in range(n):
    move_to_object_linear(objects[i+1], 0.4)
    move_to_object_linear(objects[i+1], 0.17)
    close()
    move_to_object_linear(objects[i+1], 0.4)
    move_to_object_linear(objects[0], 0.4)
    open()
remove_obstacles(objects)
moveit_commander.roscpp_shutdown()
