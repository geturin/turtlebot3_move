#! /usr/bin/env python3
import rospy
import numpy as np
import cv2,cv_bridge
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from turtlebot3_move.srv import checkPoint,checkPointRequest,checkPointResponse
 
def callback_map(mapData):
   global map,origin
 
   data = mapData.data
   data = np.array(data,np.uint8)
   width = mapData.info.width
   height = mapData.info.height
   origin = int((width/2)-mapData.info.origin.position.x)-1
   
   #for i in np.nditer(data,op_flags=['readwrite']):
       #if i == 100:
           #i[...] = 255
       #elif i == 0:
           #i[...] = 0
       #elif i == -1:
           #i[...] = 255
    
   map = np.reshape(data,(width,height))
   
 
   return 

   # map edge check
   #map = cv2.inRange(data,0,1)
   #edges = cv2.Canny(data,0,255)

   #contours, hierarchy = cv2.findContours(data,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)		
   #cv2.drawContours(data, contours, -1, (255,255,255), 5)
   #color=cv2.cvtColor(data,cv2.COLOR_GRAY2BGR)


   #cv2.imshow("contours",data)
   #cv2.waitKey()

def callback_costmap(mapData):
   global costmap

   data = mapData.data
   data = np.array(data,np.uint8)
   width = mapData.info.width
   height = mapData.info.height

   costmap = np.reshape(data,(width,height))

   #ans=check_targetPoint(map,costmap,origin,target_point=[-2,-0.5])
   #print(ans)
 
   return

def service_callback(request):
    if not isinstance(request,checkPointRequest):
       return

    x = request.pose.position.x
    y = request.pose.position.y

    response = checkPointResponse()

    response.bool.data = check_targetPoint(map,costmap,origin,target_point=[x,y])

    return response


def check_targetPoint(map,costmap,origin,target_point):
#Check the security of the target point
   pixel_x = int(target_point[0]/0.05)
   pixel_y = int(target_point[1]/0.05)
   x = int(origin + pixel_x)
   y = int(origin - pixel_y)

   if map[y][x] == 0 and costmap[y][x] <= 40:
       return True
   else:
       return False








      
rospy.init_node('targetPointCheck')
mapData=OccupancyGrid()
sub1=rospy.Subscriber('/map',OccupancyGrid,callback_map)
sub2=rospy.Subscriber("/move_base/global_costmap/costmap",OccupancyGrid,callback_costmap)

service=rospy.Service("/check",checkPoint,service_callback)



rospy.spin()


