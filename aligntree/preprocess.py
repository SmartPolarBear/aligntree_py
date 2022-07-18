import open3d as o3d
import numpy as np

from sklearn.cluster import DBSCAN

import CSF

import pyransac3d as pyrsc

from aligntree.utils import plane_dist

from TreeTool import tree_tool

from aligntree.utils import view_points


def stem_mapping(pcd: o3d.geometry.PointCloud, lmbd=0.9):
    pcd.estimate_normals()
    pcd.colors = o3d.utility.Vector3dVector([[1, 0, 0]] * len(pcd.points))

    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)

    tt = tree_tool.TreeTool(point_cloud=points)
    tt.step_1_remove_floor()
    tt.step_2_normal_filtering()
    tt.step_3_euclidean_clustering()
    tt.step_4_group_stems()
    tt.step_5_get_ground_level_trees()
    tt.step_6_get_cylinder_tree_models()
    tt.step_7_ellipse_fit()

    tree_model_info = [[i['model'][0], i['model'][1], i['model'][2]] for i in tt.finalstems]
    stems = np.array(tree_model_info)
    print(stems)
    # view_points(stems)
    return stems
