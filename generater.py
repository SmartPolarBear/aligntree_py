# generate a pair of data

import numpy as np

import open3d as o3d

import copy

tls: o3d.geometry.PointCloud = o3d.io.read_point_cloud("data/base_cloud.pcd", format='pcd')

points = np.asarray(tls.points)
center = np.average(points, axis=0)

tls_r = copy.deepcopy(tls)
# tls_r = tls_r.translate((100, 0, 0))
# tls_r = tls_r.rotate(tls.get_rotation_matrix_from_xyz((np.pi / 8, np.pi / 12, 0)),
#                      center=center)

tls_r = tls_r.transform(np.asarray([[0.86, -0.5, 0, 0], [0.5, 0.86, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))

o3d.visualization.draw_geometries([tls, tls_r])

o3d.io.write_point_cloud("data/base_cloud_rotated.pcd", tls_r)
