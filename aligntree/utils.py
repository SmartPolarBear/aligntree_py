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

    cors_edges = list()
    set_all = {0, 1, 2, 3, 4, 5}
    for ea, eb in zip(sidx_a, sidx_b):
        set_eq = {EDGE_MAP[ea][0], EDGE_MAP[ea][1], EDGE_MAP[eb][0] + 3, EDGE_MAP[eb][1] + 3}
        intersect = set_all - set_eq
        intersect = list(intersect)
        l = intersect[0]
        r = intersect[1]
        if l > 2:
            l -= 3
        if r > 2:
            r -= 3

        cors_edges.append((l, r))

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

def compute_rigid(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t

