import open3d as o3d

# Load point cloud
pcd = o3d.io.read_point_cloud("pointcloud.pcd")

#visualize the original point cloud
# o3d.visualization.draw_geometries([pcd], window_name="Original Point Cloud")

# Downsample
voxel_size = 30
downsampled_pcd = pcd.voxel_down_sample(voxel_size)

# Visualize downsampled point cloud
# o3d.visualization.draw_geometries([downsampled_pcd], window_name="Downsampled Point Cloud")

# Remove outliers
cl, ind = downsampled_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
clean_pcd = downsampled_pcd.select_by_index(ind)

# Visualize cleaned point cloud
# o3d.visualization.draw_geometries([clean_pcd], window_name="Cleaned Point Cloud")

# Estimate normals
clean_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=20))
clean_pcd.orient_normals_consistent_tangent_plane(k=10)

# Generate mesh
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(clean_pcd, depth=9)
# Remove low-density vertices (to remove the big plane)
import numpy as np
vertices_to_remove = densities < np.quantile(densities, 0.50)  # Remove lowest 1%
mesh.remove_vertices_by_mask(vertices_to_remove)
# Post-process mesh
# simplified_mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=10000)
smoothed_mesh = mesh.filter_smooth_taubin(number_of_iterations=10)

# Save mesh
o3d.io.write_triangle_mesh("output_mesh.ply", mesh)

# Visualize
o3d.visualization.draw_geometries([mesh])