import open3d as o3d
import numpy as np
import itertools
from tqdm import tqdm

from aligntree.utils import point_to_pcd, get_d_local, get_d_global


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
    cpts_ret = list()
    for ti, t in tqdm(enumerate(t_tgt)):
        min_dlocal = np.inf
        match_t = -1
        c_edges = None
        for si, s in enumerate(t_src):
            matched, dlocal, ce = get_d_local(t, s, epsilon_edge)
            if matched:
                if dlocal < min_dlocal:
                    min_dlocal = dlocal
                    match_t = si
                    c_edges = ce

        if match_t >= 0:
            ret.append((ti, match_t))
            cpts_ret.append(c_edges)

    return ret, cpts_ret


def triangle_global_matching(t_src, t_tgt, local, epsilon_edge):
    largest = list()
    for uiidx, ui in tqdm(enumerate(local)):
        consensus_set = list()
        consensus_set.append(uiidx)
        for ujidx, uj in enumerate(local):
            if uiidx == ujidx:
                continue
            # print(ui, uj)
            d_global = get_d_global((t_tgt[ui[0]], t_src[ui[1]]), (t_tgt[uj[0]], t_src[uj[1]]))
            if d_global < epsilon_edge:
                consensus_set.append(ujidx)

        if len(consensus_set) > len(largest):
            largest = consensus_set

    return largest
