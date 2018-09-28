#!/usr/bin/env python

import rospy
from phoxi_camera.srv import *
from std_srvs.srv import *
from time import sleep
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud

def depthIm_callback(depthIm):
    rospy.loginfo('Get DepthIm')

def pcl_callback(point_cloud):
    rospy.loginfo('Get Point Cloud')

document_path = "/home/bionicdl/calibration_images/"

if __name__ == '__main__':
    
    # initialize the ROS nod
    rospy.init_node('Photoneo_Control_Node')

    # connect to phoxicamera
    message = rospy.ServiceProxy('phoxi_camera/connect_camera', ConnectCamera)('1711004').message
    sleep(2)
    rospy.loginfo(message)
    im_msg = rospy.ServiceProxy('phoxi_camera/get_frame', GetFrame)(-1).message # obtain a new frame from the camera
    # Subscribe depth_map and pointcloud 
    sub_depthMap = rospy.Subscriber("phoxi_camera/depth_map", Image, depthIm_callback)
    sub_pointCloud = rospy.Subscriber("phoxi_camera/pointcloud", PointCloud2, pcl_callback)
    rospy.spin()

