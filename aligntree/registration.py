import open3d as o3d

from aligntree.preprocess import stem_mapping


def registrate(src_pcd, target_pcd):
    stems = stem_mapping(src_pcd)
    print(stems)
