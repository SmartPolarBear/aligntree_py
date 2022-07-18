import numpy as np
import open3d as o3d

from scipy.spatial import distance


def point_to_pcd(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def plane_dist(q, p, n):
    """
    :param q: a point
    :param p: a point on plane
    :param n: the normal vector of the plane
    :return:
    """
    pq = q - p
    dot = np.dot(pq, n)
    ret = dot / np.linalg.norm(n)
    if not np.isfinite([ret]).all():
        return 0x7fffffff
    return ret


def view_points(points):
    original = point_to_pcd(points)
    original.colors = o3d.utility.Vector3dVector([[0, 0, 1]] * len(points))

    o3d.visualization.draw_geometries([original])


def distances_and_edges(a):
    dists_a = distance.squareform(distance.pdist(a))
    edges_a = np.array([dists_a[0, 1], dists_a[0, 2], dists_a[1, 2]])
    sidx_a = np.argsort(edges_a)
    edges_a = np.sort(edges_a)
    return edges_a, sidx_a


def get_d_local(a, b, epsilon_edge):
    EDGE_MAP = {0: [0, 1], 1: [0, 2], 2: [1, 2]}

    edges_a, sidx_a = distances_and_edges(a)

    edges_b, sidx_b = distances_and_edges(b)

    ret = 0.0
    for ea, eb in zip(edges_a, edges_b):
        if np.abs(ea - eb) > epsilon_edge:
            return False, np.inf, None
        ret += np.abs(ea - eb)

    cors_edges = set()
    for ea, eb in zip(sidx_a, sidx_b):
        cors_edges.add((EDGE_MAP[ea][0], EDGE_MAP[eb][0]))
        cors_edges.add((EDGE_MAP[ea][1], EDGE_MAP[eb][1]))
    cors_edges = list(cors_edges)
    cors_edges = list([(a[p[0]], b[p[1]]) for p in cors_edges])

    return True, ret, cors_edges


def get_d_global(p1, p2):
    ga = list()
    for i in range(3):
        for j in range(3):
            ga.append(np.linalg.norm(p1[0][i] - p2[0][j]))

    gb = list()
    for i in range(3):
        for j in range(3):
            gb.append(np.linalg.norm(p1[1][i] - p2[1][j]))

    ret = 0.0
    for a in ga:
        for b in gb:
            dt = np.abs(a - b)
            if dt > ret:
                ret = dt

    return ret


def compute_rigid(P, Q):
    assert P.shape == Q.shape
    n, dim = P.shape

    centeredP = P - P.mean(axis=0)
    centeredQ = Q - Q.mean(axis=0)

    C = np.dot(np.transpose(centeredP), centeredQ) / n

    V, S, W = np.linalg.svd(C)
    d = (np.linalg.det(V) * np.linalg.det(W)) < 0.0

    if d:
        S[-1] = -S[-1]
        V[:, -1] = -V[:, -1]

    R = np.dot(V, W)

    varP = np.var(a1, axis=0).sum()
    c = 1 / varP * np.sum(S)  # scale factor

    t = Q.mean(axis=0) - P.mean(axis=0).dot(c * R)

    return c, R, t
