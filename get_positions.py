#!/usr/bin/env python
import rospy
import numpy as np # For random numbers
import time
 
 
from std_msgs.msg import String
from gazebo_msgs import msg 
 
class position:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

#Create callback. This is what happens when a new message is received
def sub_cal(msg):
    n = len(msg.name)-2
    global objects
    objects = [position() for i in range(n)] 
    for i in range(n-1,-1,-1):
        objects[n-1-i].x = msg.pose[i+2].position.x
        objects[n-1-i].y = msg.pose[i+2].position.y
        objects[n-1-i].z = msg.pose[i+2].position.z


#Initialize publisher
rospy.Subscriber('gazebo/model_states', msg.ModelStates, sub_cal, queue_size=1000)

rospy.init_node('gazebo_listener')
# rospy.spin()
while(1):
    n = len(objects)
    print(n)
    if(n > 0):
        print('bucket\nx = {}\ny = {}\nz = {}'.format(objects[0].x, objects[0].y, objects[0].z))
        for i in range(1,n):
            print('cube{}\nx = {}\ny = {}\nz = {}'.format(n-1-i, objects[i].x, objects[i].y, objects[i].z))
    rospy.sleep(2)