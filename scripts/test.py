#! /usr/bin/env python3
import rospy
from turtlebot3_move.srv import checkPoint,checkPointRequest,checkPointResponse
from geometry_msgs.msg import Point

client = rospy.ServiceProxy("/check",checkPoint)
request = checkPointRequest()

request.pose.position.x=-4.0
request.pose.position.y=-0.5

response=client.call(request)

print(response.bool.data)
