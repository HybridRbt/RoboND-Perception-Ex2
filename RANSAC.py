# Import PCL module
import pcl

# Load Point Cloud file
#cloud = pcl.load_XYZRGB('tabletop.pcd')

def voxel_downsampling(pcl_data):
    # Voxel Grid filtering
    # Create a VoxelGrid filter object for out input point cloud
    vox = pcl_data.make_voxel_grid_filter()

    # choose a voxel (leaf) size
    LEAF_SIZE = 0.01

    # Set voxel size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # call the filter funciton to obtain the resultant downsampled point cloud
    pcl_filtered = vox.filter()
    return pcl_filtered

def passthrough_filtering(pcl_data):
    # PassThrough filtering
    # Create a PassThrough filter objects
    passthrough = pcl_data.make_passthrough_filter()

    # Assign axis and range to the passthrough filter objects
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)

    # set the limits
    axis_min = 0.6  # this retains the table and the objects
    axis_max = 1.1

    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally, use the filter function to obtain the resultant point cloud
    pcl_filtered = passthrough.filter()
    return pcl_filtered

def plane_fitting(pcl_data):
    # RANSAC plane segmentation
    # Create the segmentation object
    seg = pcl_data.make_segmenter()

    # Set the model you wish to filter
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Note: in lesson 3-15, the quizz for this number claims it's 0.01
    # but in that case the front of the table will show, and will keep showing
    # until increased to 0.034. but in this case the bottom of the bowl will be
    # cut. Need to figure out which number to take.
    max_distance = 0.035 # this leaves only the table
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    return inliers

def extract_inliers(inliers, pcl_data):
    # Extract inliers
    extracted_inliers = pcl_data.extract(inliers, negative=False)
    return extracted_inliers

def extract_outliers(inliers, pcl_data):
    # Extract outliers
    extracted_outliers = pcl_data.extract(inliers, negative=True)
    return extracted_outliers
