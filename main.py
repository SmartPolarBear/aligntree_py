from aligntree.registration import registrate
from converter import read_laz_point_cloud

import open3d as o3d

pcd = o3d.io.read_point_cloud("data/ply/als.ply", format='ply')
o3d.visualization.draw_geometries([pcd])

