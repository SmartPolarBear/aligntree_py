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


def triangle_same(a, b, epsilon_edge):
    dists_a = distance.squareform(distance.pdist(a))
    edges_a = np.sort(np.array([dists_a[0, 1], dists_a[0, 2], dists_a[1, 2]]))

    dists_b = distance.squareform(distance.pdist(b))
    edges_b = np.sort(np.array([dists_b[0, 1], dists_b[0, 2], dists_b[1, 2]]))

    ret = 0.0
    for ea, eb in zip(edges_a, edges_b):
        if np.abs(ea - eb) > epsilon_edge:
            return False, np.inf
        ret += np.abs(ea - eb)

    return True, ret
