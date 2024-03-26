import open3d as o3d
import numpy as np

# Load the PCD file
pcd = o3d.io.read_point_cloud("jfe_1st_to_3rd.pcd")

# Estimate normals
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=50))

# Cluster the point cloud using DBSCAN
labels = np.array(pcd.cluster_dbscan(eps=0.2, min_points=10, print_progress=True))
max_label = labels.max()

print(f"Point Cloud has {max_label + 1} clusters")

meshes = []

# Generate a Convex Hull for each cluster, ensuring each has at least 4 points
for i in range(max_label + 1):
    cluster_indices = np.where(labels == i)[0]
    if len(cluster_indices) < 4:
        print(f"Skipping cluster {i} with less than 4 points.")
        continue  # Skip clusters with less than 4 points
    cluster = pcd.select_by_index(cluster_indices)
    try:
        hull, _ = cluster.compute_convex_hull()
        hull.paint_uniform_color(np.random.rand(3))
        meshes.append(hull)
    except RuntimeError as e:
        print(f"Failed to compute convex hull for cluster {i}: {e}")

# Save or visualize the Convex Hulls
o3d.visualization.draw_geometries(meshes)

# Initialize lists to hold the combined vertices and triangles
combined_vertices = np.empty((0, 3), dtype=float)
combined_triangles = np.empty((0, 3), dtype=int)
vertex_offset = 0  # Keep track of the offset for triangle indices

for mesh in meshes:
    # Get vertices and triangles from the current mesh
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    
    # Offset the triangles' indices and append to the combined list
    triangles_offset = triangles + vertex_offset
    combined_vertices = np.vstack((combined_vertices, vertices))
    combined_triangles = np.vstack((combined_triangles, triangles_offset))
    
    # Update the vertex offset for the next mesh
    vertex_offset += len(vertices)

# Create a new mesh with the combined vertices and triangles
combined_mesh = o3d.geometry.TriangleMesh()
combined_mesh.vertices = o3d.utility.Vector3dVector(combined_vertices)
combined_mesh.triangles = o3d.utility.Vector3iVector(combined_triangles)

# Optionally, compute normals for the new mesh
combined_mesh.compute_vertex_normals()

# Save the combined mesh as a single STL file
o3d.io.write_triangle_mesh("combined_output_mesh.stl", combined_mesh)
print("Saved combined mesh to 'combined_output_mesh.stl'")