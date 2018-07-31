#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data

    # TODO: Voxel Grid Downsampling

    # TODO: PassThrough Filter

    # TODO: RANSAC Plane Segmentation

    # TODO: Extract inliers and outliers

    # TODO: Euclidean Clustering

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # TODO: Convert PCL data to ROS messages

    # TODO: Publish ROS messages
    pub_objects.publish(pcl_msg)
    pub_table.publish(pcl_msg)
# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)

        # Grab the points for the cluster

        # Compute the associated feature vector

        # Make the prediction

        # Publish a label into RViz

        # Add the detected object to the list of detected objects.

    # Publish the list of detected objects

if __name__ == '__main__':

    #ROS node initialization
    
    #It adds a random number to the end of your node's name, to make it unique. 
    #Unique names are more important for nodes like drivers, where it is an error if more than one is running. 
    #If two nodes with the same name are detected on a ROS graph, the older node is shutdown.
    rospy.init_node('clustering',anonymous=True)
    
    #Create Subscribers
    sub = rospy.Subscriber("/sensor_stick/point_cloud",PointCloud2,pcl_callback,queue_size=10)

    #Create Publishers for objects and table
    pub_objects = rospy.Publisher("/pcl_objects",PointCloud2,queue_size=1)
    pub_table = rospy.Publisher("/pcl_table",PointCloud2,queue_size=1)
    
    #Load Model From disk
    
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()