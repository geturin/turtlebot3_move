#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped  
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from turtlebot3_move.srv import checkPoint,checkPointRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
import numpy as np
from simple_tf import *


class move_base(object):

    def __init__(self,frame) -> None:
        self.frame = frame
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
        self.move_base.wait_for_server(rospy.Duration(5.0))  
        rospy.loginfo("Connected to move base server")  
        return

    def set_goal(self,pose):
        self.goal = MoveBaseGoal()  
        self.goal.target_pose.header.frame_id = self.frame  
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = pose
        rospy.loginfo("Sending goal")  
        self.move_base.send_goal(self.goal)  
        return


    def wait_success(self):
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))       
        if not finished_within_time:  
            self.move_base.cancel_goal()  
            rospy.loginfo("Timed out achieving goal")  
        else:  
            state = self.move_base.get_state()  
            if state == GoalStatus.SUCCEEDED:  
                rospy.loginfo("Goal succeeded!")
            else:  
                rospy.loginfo("Goal failed！ ")  
        
        return


class random_goal(object):

    def __init__(self,move_distance) -> None:
        self.theta = np.arange(0,360,10)
        self.move_distance = move_distance
        self.client = rospy.ServiceProxy("/check",checkPoint)
        self.tf = simpele_TF("base_footprint", "map") 
        self.posestamped = PoseStamped()
        return

    def polar_to_Cartesian(self,distance,theta):
        y = distance * np.sin(theta*np.pi/180)
        x = distance * np.cos(theta*np.pi/180)
        return x,y

    def random_pose(self):
        np.random.shuffle(self.theta)
        distance = self.move_distance

        while True:
            for i in self.theta:
                x,y = self.polar_to_Cartesian(distance,i)
                self.posestamped.pose.position.x = x
                self.posestamped.pose.position.y = y
                self.posestamped = self.tf.transform_pose(self.posestamped)
                response=self.client.call(self.posestamped.pose)
                if response.bool.data == True:
                    return self.posestamped.pose

            distance -=0.3
    
    



