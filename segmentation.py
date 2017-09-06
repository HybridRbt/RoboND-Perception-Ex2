#!/usr/bin/env python

# Import modules
from pcl_helper import *
from RANSAC import *

# Define functions as required
def clustering(pcl_data):
    # Create k-d tree
    white_cloud = XYZRGB_to_XYZ(pcl_data)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()

    # Set tolerances for distance threshold
    ec.set_ClusterTolerance(0.001)

    # Set minimum cluster size
    ec.set_MinClusterSize(10)

    # Set maximum cluster size
    ec.set_MaxClusterSize(250)

    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)

    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    return cluster_indices

def color_clusters(cluster_indices):
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    # create a list for the colored cluster points
    color_cluster_point_list = []

    # traverse the indices and append to the list
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    # Create new cloud containing all clusters colored
    cluster_cloud_colored = pcl.PointCloud_PointXYZRGB()
    cluster_cloud_colored.from_list(color_cluster_point_list)
    return cluster_cloud_colored

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    # Voxel Grid Downsampling
    pcl_downsampled = voxel_downsampling(pcl_data)

    # PassThrough Filter
    pcl_passed = passthrough_filtering(pcl_downsampled)

    # RANSAC Plane Segmentation
    inliers = plane_fitting(pcl_passed)

    # Extract inliers and outliers
    # inliers: table
    # outliers: objects
    cloud_table = extract_inliers(inliers, pcl_passed)
    cloud_objects = extract_outliers(inliers, pcl_passed)

    # Euclidean Clustering
    cluster_indices = clustering(cloud_objects)

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_cloud_colored = color_clusters(cluster_indices)

    # Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud_colored = pcl_to_ros(cluster_cloud_colored)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud_colored)
    
if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
