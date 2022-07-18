import open3d as o3d
import numpy as np

from tqdm import tqdm

from aligntree.preprocess import stem_mapping
from aligntree.stems import construct_triangles, triangle_local_matching, triangle_global_matching
from aligntree.utils import compute_rigid


def registrate(src_pcd, target_pcd, k=5, epsilon_edge=5):
    stems_src = stem_mapping(src_pcd)
    stems_tgt = stem_mapping(target_pcd)
    print("stem points:", len(stems_src), len(stems_tgt))

    triangles_src = construct_triangles(stems_src, k)
    triangles_tgt = construct_triangles(stems_tgt, k)

    print("triangles:", len(triangles_src), len(triangles_tgt))

    local_matched, c_pts = triangle_local_matching(triangles_src, triangles_tgt, epsilon_edge=epsilon_edge)

    print("local matched:", len(local_matched))

    largest_consensus = triangle_global_matching(triangles_src, triangles_tgt, local_matched, epsilon_edge=epsilon_edge)

    print("global matched:", len(largest_consensus))

    matched_pts = list()
    for c in tqdm(largest_consensus):
        matched_pts += c_pts[c]

    p = np.array([m[0] for m in matched_pts])
    q = np.array([m[1] for m in matched_pts])

    r, t = compute_rigid(p, q)

    return r, t
