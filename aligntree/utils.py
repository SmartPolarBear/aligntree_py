import numpy as np
import open3d as o3d


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
    original = o3d.geometry.PointCloud()
    original.points = o3d.utility.Vector3dVector(points)
    original.colors = o3d.utility.Vector3dVector([[0, 0, 1]] * len(points))

    o3d.visualization.draw_geometries([original])
