import open3d as o3d

def remove_ground_plane(input_file, output_file, distance_threshold=0.15, ransac_n=3, num_iterations=100):
    # Read point cloud from input file
    cloud = o3d.io.read_point_cloud(input_file)

    # Perform plane segmentation using RANSAC
    plane_model, inliers = cloud.segment_plane(distance_threshold, ransac_n, num_iterations)

    # Extract ground plane points
    ground_points = cloud.select_by_index(inliers, invert=True)

    # Save the ground points to a new PCD file
    o3d.io.write_point_cloud(output_file, ground_points)
# Example usage:
input_pcd_file = "my_map_fastlio.pcd"
output_pcd_file = "filtered_cloud.pcd"
remove_ground_plane(input_pcd_file, output_pcd_file)
