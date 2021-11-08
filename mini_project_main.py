#!/usr/bin/env python

import rospy

import listner
import move_gripper
import move_arm
import manage_obstacles
import sys

rospy.init_node('main_control_node')
move_arm.init_movement_commander()
move_arm.move_to_start()

listner.initSubscriber()
objects = listner.get_positions()

move_gripper.open()

cubes_in_bucket = 0
n = len(objects) - 1

while cubes_in_bucket < n:
    for i in range(n):
        bucket_width = 0.12
        if ((objects[i+1].x > objects[0].x - bucket_width and 
            objects[i+1].x < objects[0].x + bucket_width) and
            (objects[i+1].y > objects[0].y - bucket_width and 
             objects[i+1].y < objects[0].y + bucket_width)): 
            cubes_in_bucket += 1
            print "cube {} is already in the bucket".format(n-i-1)
            continue
        print "collecting cube {}".format(n-i-1)
        
        move_arm.init_obstacles(objects)
        result = 1
        attempts = 0
        while (result == 1 and attempts < 2): 
            attempts +=1
            result = move_arm.move_group_python_interface(objects[i+1].x, objects[i+1].y, objects[i+1].z+0.25)
            if result == 1: continue
            else: result = 1
            result = move_arm.move_group_python_interface(objects[i+1].x, objects[i+1].y, objects[i+1].z+0.17)
        if attempts == 2:
            print "skipping cube"
            continue
        
        move_gripper.close()

        result = 1
        attempts = 0
        while (result == 1 and attempts < 5): 
            result = move_arm.move_group_python_interface(objects[0].x, objects[0].y, objects[0].z + 0.5)
            attempts += 1
        if attempts == 5:
            sys.exit()

        move_gripper.open()
        #renew object positions
        move_arm.remove_obstacles(objects)
        objects = listner.get_positions()
        

move_arm.close_movement_commander()

    # move_gripper.__execute__(0.7)
    # rospy.sleep(2)
    # move_gripper.open()
