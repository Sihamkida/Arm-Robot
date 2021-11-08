#!/usr/bin/env python

import rospy

import listner
import move_gripper
import move_arm

rospy.init_node('main_control_node')
move_arm.init_movement_commander()
listner.initSubscriber()
objects = listner.get_positions()


move_gripper.open()
rospy.sleep(2)

move_arm.move_group_python_interface(objects[0].x, objects[0].y, objects[0].z + 0.6)




# move_gripper.__execute__(0.7)
# rospy.sleep(2)
# move_gripper.open()