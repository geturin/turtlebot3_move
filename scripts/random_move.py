#! /usr/bin/env python3
import rospy
from simple_tf import *
from random_target import *


def main():
    move_distance = 1.8
    rospy.init_node('random_goal', anonymous=True)  
    move = move_base("map") 
    target = random_goal(move_distance)
    while not rospy.is_shutdown():
        goal = target.random_pose()
        move.set_goal(goal)
        move.wait_success()

    
if __name__ == '__main__':
    main()