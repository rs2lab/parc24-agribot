import open3d as o3d
cloud = o3d.io.read_point_cloud("./clouds/cloud_1.pcd")
o3d.visualization.draw_geometries([cloud])
