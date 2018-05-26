#!/usr/bin/env python

# Import modules
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

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    print ('make_yaml_dict', yaml_dict)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

def getStatisticalOutlierFilter(cloud, mean = 50, scale = 1.0):
    #create filter object
    outlier_filter = cloud.make_statistical_outlier_filter()
    #set the number of neighbouring points to analyze for any given point
    outlier_filter.set_mean_k(mean)
    #set Threshold scale factor
    outlier_filter.set_std_dev_mul_thresh(scale)
    return outlier_filter.filter()

def getVoxFilter(cloud, leaf_size = 0.01):
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud.make_voxel_grid_filter()

    #set the voxel (or leaf size)
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)

    # Call the filter function to obtain the resultant downsampled point cloud
    return vox.filter()

def getPassThroughFilter(cloud, filter_axis = 'z', 
                         axis_min = 0.7, axis_max = 1.1):
    # Create a PassThrough filter object
    passthrough = cloud.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud
    return passthrough.filter()

def getInliersUsingRANSAC(cloud, max_distance = 0.01):
    seg = cloud.make_segmenter()

    #set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance
    # for segmenting the table
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    return seg.segment()

def getClusterIndices(cloud, clustertolerance = 0.015, minclustersize = 20,
                      maxclustersize = 1500):
    #create a cluster extraction object
    ec = cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects
    ec.set_ClusterTolerance(clustertolerance)
    ec.set_MinClusterSize(minclustersize)
    ec.set_MaxClusterSize(maxclustersize)
    # Search the k-d tree for clusters
    tree = cloud.make_kdtree()
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    return ec.Extract()

def getSegmentedObjectColorList(cloud, cluster_indices):
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([cloud[indice][0],
                                             cloud[indice][1],
                                             cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    return color_cluster_point_list
 
# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
# Exercise-2 TODOs:
    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg) 
    # TODO: Statistical Outlier Filtering
    cloud_filtered = getStatisticalOutlierFilter(cloud, 50, 0.5)

    # TODO: Voxel Grid Downsampling
    cloud_filtered = getVoxFilter(cloud_filtered)

    # TODO: PassThrough Filter
    cloud_filtered = getPassThroughFilter(cloud_filtered, 'x', 0, 1.5)
    cloud_filtered = getPassThroughFilter(cloud_filtered, 'y', -0.5, 0.5)
    cloud_filtered = getPassThroughFilter(cloud_filtered, 'z', 0.6, 1.2)
    # TODO: RANSAC Plane Segmentation
    inliers, coefficients = getInliersUsingRANSAC(cloud_filtered, 0.01)

    # TODO: Extract inliers and outliers
    #Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative = False)

    #Extract outliers
    cloud_objects = cloud_filtered.extract(inliers, negative = True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_indices = getClusterIndices(white_cloud, 0.015, 20, 1500)
    color_cluster_point_list = getSegmentedObjectColorList(white_cloud, cluster_indices)
    # convert list of point cloud features (x,y,z, rgb) to a point cloud
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:
    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        #Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        #Convert the cluster from pcl to ros
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        #extract histogam features
        colorhists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        normhists = compute_normal_histograms(normals)
        features = np.concatenate((colorhists, normhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(features.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(detected_list):

    # TODO: Initialize variables
    output = []
    OBJECT_NAME = String()
    WHICH_ARM = String()
    PICK_POSE = Pose()
    PLACE_POSE = Pose()
    TEST_SCENE_NUM = Int32()
    TEST_SCENE_NUM.data = 3

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    drop_list_param   = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables
    #set up object to group mapping
    object_name_to_group_map = {}
    for obj_params in object_list_param:
        name = obj_params['name']
        group = obj_params['group']
        object_name_to_group_map[name] = group

    #setup dropbox to position mapping
    dropbox_name_to_position_map = {}
    dropbox_group_to_name_map = {}
    for box in drop_list_param:
        name = box['name']
        position = box['position']
        group = box['group']
        dropbox_name_to_position_map[name] = position
        dropbox_group_to_name_map[group] = name

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for detectedObject in detected_list:
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        OBJECT_NAME.data = detectedObject.label
        print 'processing object name', OBJECT_NAME.data
        points_arr = ros_to_pcl(detectedObject.cloud).to_array()
        centroid = [np.asscalar(x) for x in np.mean(points_arr, axis=0)[:3]]

        PICK_POSE.position.x = centroid[0]
        PICK_POSE.position.y = centroid[1]
        PICK_POSE.position.z = centroid[2]

        PICK_POSE.orientation.x = 0
        PICK_POSE.orientation.y = 0
        PICK_POSE.orientation.z = 0
        PICK_POSE.orientation.w = 0
 
        # TODO: Create 'place_pose' for the object
        # TODO: Assign the arm to be used for pick_place
        group_name = object_name_to_group_map[OBJECT_NAME.data]
        WHICH_ARM.data = dropbox_group_to_name_map[group_name]

        PLACE_POSE.position.x = dropbox_name_to_position_map[WHICH_ARM.data][0]
        PLACE_POSE.position.y = dropbox_name_to_position_map[WHICH_ARM.data][1]
        PLACE_POSE.position.z = dropbox_name_to_position_map[WHICH_ARM.data][2]

        PLACE_POSE.orientation.x = 0
        PLACE_POSE.orientation.y = 0
        PLACE_POSE.orientation.z = 0
        PLACE_POSE.orientation.w = 0
 
        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        output.append(make_yaml_dict(TEST_SCENE_NUM, WHICH_ARM, OBJECT_NAME,
                      PICK_POSE, PLACE_POSE))
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')
        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    yaml_filename = 'output_{}'.format(TEST_SCENE_NUM.data)
    send_to_yaml(yaml_filename, output)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('recognition', anonymous = True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2,
                               pcl_callback, queue_size=1)

    # TODO: Create Publishers
    object_markers_pub = rospy.Publisher('/object_markers', Marker,
                                         queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects',
                                           DetectedObjectsArray, queue_size=1)
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2,
                                           queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
