#! /usr/bin/env python3
from matplotlib.pyplot import thetagrids
import rospy
import actionlib
from actionlib_msgs.msg import * 
import numpy as np
from nav_msgs.msg import Odometry
from move_base_msgs.msg  import MoveBaseAction,MoveBaseGoal,MoveBaseActionGoal
from turtlebot3_move.srv import checkPoint,checkPointRequest,checkPointResponse
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist ,PoseStamped  
import tf2_geometry_msgs
import tf2_ros


def polar_to_Cartesian(distance,theta):
    y = distance * np.sin(theta*np.pi/180)
    x = distance * np.cos(theta*np.pi/180)

    return x,y


def random_targetPoint(move_distance,theta,client,tf_buffer):
    distance = move_distance
    request = checkPointRequest()
    pose_odomFrame = PoseStamped()

    np.random.shuffle(theta)
    odom_tf_map = get_transformation("base_footprint", "map",tf_buffer)

    while True:


        for i in theta:
            x,y = polar_to_Cartesian(distance,i)

            #request.pose.position.x = x
            #request.pose.position.y = y
            pose_odomFrame.pose.position.x = x
            pose_odomFrame.pose.position.y = y

            pose_mapframe = transform_pose(odom_tf_map,pose_odomFrame)

            request.pose.position = pose_mapframe.pose.position
            response=client.call(request)

            if response.bool.data == True:
                return pose_mapframe.pose.position

        distance -= 0.3


def get_transformation(source_frame, target_frame,tf_buffer,
                       tf_cache_duration=2.0):


    # get the tf at first available time
    try:
        transformation = tf_buffer.lookup_transform(target_frame,
                source_frame, rospy.Time(0), rospy.Duration(2))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s'
                     % source_frame, target_frame)
    return transformation

def transform_pose(transformation, pose):
    tfpose = \
        tf2_geometry_msgs.do_transform_pose(pose,transformation)
    return tfpose

    

def random_goal_target(move_base, client , move_distance , theta,tf_buffer): 

    goal = MoveBaseGoal()  
    goal.target_pose.header.frame_id = 'map'  
    goal.target_pose.header.stamp = rospy.Time.now()  


    goal.target_pose.pose.position =  random_targetPoint(move_distance,theta,client,tf_buffer)
    goal.target_pose.pose.orientation.w = 1.0

    # 3 send goal target
    rospy.loginfo("Sending goal")  
    move_base.send_goal(goal)

    #  4 wait for goal  
    finished_within_time = move_base.wait_for_result(rospy.Duration(300))       
    if not finished_within_time:  
        move_base.cancel_goal()  
        rospy.loginfo("Timed out achieving goal")  
    else:  
        state = move_base.get_state()  
        if state == GoalStatus.SUCCEEDED:  
            rospy.loginfo("Goal succeeded!")
            #loop
            random_goal_target(move_base, client , move_distance , theta , tf_buffer)
        else:  
            rospy.loginfo("Goal failed！ ") 
            random_goal_target(move_base, client , move_distance , theta , tf_buffer)



def main():
    move_distance = 1.8
    theta = np.arange(0,360,10)

     # 1 init ros node ,move_base server  ,tf2
    rospy.init_node('random_goal', anonymous=True)  
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
    move_base.wait_for_server(rospy.Duration(5.0))
    client = rospy.ServiceProxy("/check",checkPoint)  
    rospy.loginfo("Connected to move base server") 
    tf_buffer = tf2_ros.Buffer(rospy.Duration(2))
    tf2_ros.TransformListener(tf_buffer)

    # 2 random goal target and send it
    random_goal_target(move_base, client , move_distance , theta,tf_buffer)
    
     


if __name__ == '__main__':
    main()