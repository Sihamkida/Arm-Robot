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
import math
 
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
  group.allow_replanning(True)  
  group.set_planning_time(5.)
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
  #group.set_planning_time(0.0)
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)
  group.set_max_velocity_scaling_factor(1.0)
  group.set_max_acceleration_scaling_factor(1.0)

def close_movement_commander():
    moveit_commander.roscpp_shutdown()


def move_to_start():
    print("move to starting position")
    pose_goal = group.get_current_pose().pose
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
    pose_goal.position.x =0.40
    pose_goal.position.y =-0.10
    pose_goal.position.z =1.35  
    group.set_pose_target(pose_goal)
    plan1 = group.plan()
    group.go(wait=True)


def move_group_python_interface(x, y, z):
  ## Planning to a Pose goal
  print "starting pose"
  print group.get_current_pose()

  pose_goal = group.get_current_pose().pose
  pose_endgoal = pose_goal
  waypoints = []
 
  waypoints.append(pose_goal)

  pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
  pose_goal.position.x = x
  pose_goal.position.y = y
  pose_goal.position.z = z
  waypoints.append(pose_goal)
  print "target pose"
  print pose_goal

  print "============ Generating plan 1"
  #createcartesian  plan
  fraction = 0
  attempts = 0
  while (fraction < 0.99 and attempts < 10):
    (plan1, fraction) = group.compute_cartesian_path(       
                            waypoints,   # waypoints to follow
                            0.01,        # eef_step
                            0.0)         # jump_threshold
    print "fraction %s" % fraction
    attempts += 1
  
  if fraction < 0.40:
    move_to_start()
    return 1

  plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
 
 
  # print "============ Waiting while RVIZ displays plan1..."
  # rospy.sleep(0.5)
 
 
  # ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
  # print "============ Visualizing plan1"
  # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  # display_trajectory.trajectory_start = robot.get_current_state()
  # display_trajectory.trajectory.append(plan1)
  # display_trajectory_publisher.publish(display_trajectory);
  # print "============ Waiting while plan1 is visualized (again)..."
  # rospy.sleep(2.)
 
  #If we're coming from another script we might want to remove the objects
  if "table" in scene.get_known_object_names():
    scene.remove_world_object("table")
  if "table2" in scene.get_known_object_names():
    scene.remove_world_object("table2")
  if "groundplane" in scene.get_known_object_names():
    scene.remove_world_object("groundplane")
 
  # Moving to a pose goal
  group.execute(plan1,wait=True)
  rospy.sleep(1.)
  
  print "final pose"
  print group.get_current_pose()
  resX = group.get_current_pose().pose.position.x
  resY = group.get_current_pose().pose.position.y
  posX = pose_goal.position.x
  posY = pose_goal.position.y
  if (abs(resX-posX) > 0.02 or abs(resY-posY) > 0.02):
      return 1
  return 0
