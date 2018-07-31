# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')
# cloud = pcl.load("table_scene_lms400.pcd")

# Statistical outlier filter
so_filter = cloud.make_statistical_outlier_filter()
so_filter.set_mean_k(50)
so_filter.set_std_dev_mul_thresh(1)
cloud = so_filter.filter()
filename = 'statistical_outlier.pcd'
pcl.save(cloud, filename)

# Voxel Grid filter
vox = cloud.make_voxel_grid_filter()
LEAF_SIZE = 0.005
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)

# # PassThrough filter
# # Create a PassThrough filter object.
passthrough = cloud_filtered.make_passthrough_filter()

# Assign axis and range to the passthrough filter object.
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.6
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)

# # Finally use the filter function to obtain the resultant point cloud. 
cloud_filtered = passthrough.filter()

filename = 'pass_through_filtered_z.pcd'
pcl.save(cloud_filtered, filename)

# # RANSAC plane segmentation
# Create the segmentation object
seg = cloud_filtered.make_segmenter()

# Set the model you wish to fit 
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

# Max distance for a point to be considered fitting the model
# Experiment with different values for max_distance 
# for segmenting the table
max_distance = 0.04
seg.set_distance_threshold(max_distance)

# Call the segment function to obtain set of inlier indices and model coefficients
inliers, coefficients = seg.segment()

# Extract inliers
extracted_inliers = cloud_filtered.extract(inliers, negative=False)
filename = 'table_inliers_4e-2.pcd'
pcl.save(extracted_inliers, filename)


# Extract outliers
extracted_outliers = cloud_filtered.extract(inliers, negative=True)
filename = 'objects_outliers_4e-2.pcd'

# Save pcd for tabletop objects
pcl.save(extracted_outliers, filename)


