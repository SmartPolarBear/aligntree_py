from aligntree.registration import registrate
from converter import read_laz_point_cloud

import open3d as o3d

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
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


# smallest
tls: o3d.geometry.PointCloud = o3d.io.read_point_cloud("data/ply/tls.ply", format='ply')
voxel_tls = tls.voxel_down_sample(voxel_size=2000)

# smaller
als: o3d.geometry.PointCloud = o3d.io.read_point_cloud("data/ply/als.ply", format='ply')
voxel_als = als.voxel_down_sample(voxel_size=2000)

trans = o3d_registrate(tls, als, voxel_size=2000)

draw_registration_result(tls, als, trans)
