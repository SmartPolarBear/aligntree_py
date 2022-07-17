# generate a pair of data

import numpy as np

import open3d as o3d

import copy

tls: o3d.geometry.PointCloud = o3d.io.read_point_cloud("data/base_cloud.pcd", format='pcd')
tls_r = copy.deepcopy(tls).translate((1, 0, 0))
tls_r.rotate(tls.get_rotation_matrix_from_xyz((np.pi / 8, np.pi / 12, np.pi / 5)),
             center=(0, 0, 0))

o3d.io.write_point_cloud("data/base_cloud_rotated.pcd", tls_r)

o3d.visualization.draw_geometries([tls, tls_r])
