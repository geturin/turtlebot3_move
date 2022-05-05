#!/usr/bin/python3.8
import rospy
import tf2_geometry_msgs
import tf2_ros

class simpele_TF(object):
    
    def __init__(self,source_frame,target_frame):
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.creat_listener()
        return



    def creat_listener(self):
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(2))
        tf2_ros.TransformListener(self.tf_buffer)
        return

    def get_transformation(self):
    # get the tf at first available time
        try:
            self.transformation = self.tf_buffer.lookup_transform(self.target_frame,
                    self.source_frame, rospy.Time(0), rospy.Duration(2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr('Unable to find the transformation from %s to %s'
                            % self.source_frame, self.target_frame)
        return 

    def transform_pose(self,pose):
        self.get_transformation()   
        self.tfpose =  tf2_geometry_msgs.do_transform_pose(pose,self.transformation)
        return self.tfpose


