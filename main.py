import numpy
import numpy as np

from aligntree.registration import registrate

import open3d as o3d
import pptk

import copy

from o3d_registration import o3d_registrate


def print_all_pcds():
    # largest
    uls: o3d.geometry.PointCloud = o3d.io.read_point_cloud("data/ply/uls.ply", format='ply')
    voxel_uls = uls.voxel_down_sample(voxel_size=2000)
    voxel_uls.colors = o3d.utility.Vector3dVector([[0.9, 0, 0] for p in range(len(voxel_uls.points))])

    # smallest
    tls: o3d.geometry.PointCloud = o3d.io.read_point_cloud("data/ply/tls.ply", format='ply')
    voxel_tls = tls.voxel_down_sample(voxel_size=2000)
    voxel_tls.colors = o3d.utility.Vector3dVector([[0, 0.9, 0] for p in range(len(voxel_tls.points))])

    # smaller
    als: o3d.geometry.PointCloud = o3d.io.read_point_cloud("data/ply/als.ply", format='ply')
    voxel_als = als.voxel_down_sample(voxel_size=2000)
    voxel_als.colors = o3d.utility.Vector3dVector([[0, 0, 0.9] for p in range(len(voxel_als.points))])

    # o3d.visualization.draw_geometries([voxel_uls, voxel_tls, voxel_als])
    o3d.visualization.draw_geometries([voxel_tls, voxel_als])


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    # o3d.visualization.draw_geometries([source_temp, target_temp])

    src_points = numpy.asarray(source_temp.points)
    tgt_points = numpy.asarray(target_temp.points)

    src_colors = np.array([[1.0, 0.0, 0.0]] * len(source_temp.points))
    tgt_colors = np.array([[0.0, 1.0, 0.0]] * len(target_temp.points))

    all_points = np.concatenate([src_points, tgt_points])
    all_colors = np.concatenate([src_colors, tgt_colors])

    v = pptk.viewer(all_points, all_colors)


src: o3d.geometry.PointCloud = o3d.io.read_point_cloud("data/base_cloud_rotated.pcd", format='pcd')

tgt: o3d.geometry.PointCloud = o3d.io.read_point_cloud("data/base_cloud.pcd", format='pcd')

trans = o3d_registrate(src, tgt, voxel_size=1)
print(trans)

draw_registration_result(src, tgt, trans)
