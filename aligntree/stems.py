import open3d as o3d
import numpy as np
import itertools

from aligntree.utils import point_to_pcd, triangle_same


def construct_triangles(stems, k):
    stem_pcd = point_to_pcd(stems)
    stem_kdt = o3d.geometry.KDTreeFlann(stem_pcd)

    triangles = list()
    for i, s in enumerate(stems):
        k, idx, _ = stem_kdt.search_knn_vector_3d(query=s, knn=k)
        if k < 2:
            continue

        elements = list(idx)
        elements.append(i)
        elements = list(set(elements))

        triangles += itertools.combinations(elements, r=3)

    ret = [(stems[t[0]], stems[t[1]], stems[t[2]]) for t in triangles]
    # print(ret)
    return ret


def triangle_local_matching(t_src, t_tgt, epsilon_edge):
    ret = list()
    for ti, t in enumerate(t_tgt):
        min_dlocal = np.inf
        match_t = -1
        for si, s in enumerate(t_src):
            matched, dlocal = triangle_same(t, s, epsilon_edge)
            if matched:
                if dlocal < min_dlocal:
                    min_dlocal = dlocal
                    match_t = si

        if match_t >= 0:
            ret.append((t, match_t))

    return ret
