import open3d as o3d

# Load point cloud
pcd = o3d.io.read_point_cloud("pointcloud.pcd")

# Downsample
voxel_size = 0.02
downsampled_pcd = pcd.voxel_down_sample(voxel_size)

# Remove outliers
cl, ind = downsampled_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
clean_pcd = downsampled_pcd.select_by_index(ind)

# Estimate normals
clean_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=20))
clean_pcd.orient_normals_consistent_tangent_plane(k=10)

# Generate mesh
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(clean_pcd, depth=9)

# Post-process mesh
simplified_mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=10000)
smoothed_mesh = simplified_mesh.filter_smooth_taubin(number_of_iterations=10)

# Save mesh
o3d.io.write_triangle_mesh("output_mesh.ply", smoothed_mesh)

# Visualize
o3d.visualization.draw_geometries([smoothed_mesh])