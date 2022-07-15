from aligntree.registration import registrate
from converter import read_laz_point_cloud

import open3d as o3d

pcd = read_laz_point_cloud('data/uls.laz')
o3d.visualization.draw_geometries([pcd])
