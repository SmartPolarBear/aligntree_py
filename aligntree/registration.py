import open3d as o3d

from aligntree.preprocess import stem_mapping
from aligntree.stems import construct_triangles, triangle_local_matching


def registrate(src_pcd, target_pcd, k=5, epsilon_edge=5):
    stems_src = stem_mapping(src_pcd)
    stems_tgt = stem_mapping(target_pcd)
    print("stem points:", len(stems_src), len(stems_tgt))

    triangles_src = construct_triangles(stems_src, k)
    triangles_tgt = construct_triangles(stems_tgt, k)

    print("triangles:", len(triangles_src), len(triangles_tgt))

    local_matched = triangle_local_matching(triangles_src, triangles_tgt, epsilon_edge=epsilon_edge)

    print("local matched:", len(local_matched))
