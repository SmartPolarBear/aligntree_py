import open3d as o3d
import numpy as np

from sklearn.cluster import DBSCAN


def stem_mapping(pcd: o3d.geometry.PointCloud, lmbd=0.9):
    pcd.estimate_normals()

    points = np.asarray(pcd.points)
    normals = points = np.asarray(pcd.normals)

    assert len(points) == len(normals)
    print("Original points:", len(points))

    points = points[1 - normals[:, 2] > lmbd]
    print("Stem points identified:", len(points))

    dbscan = DBSCAN(eps=0.5)
    labels = dbscan.fit(points)
    print("Clustered into {} groups".format(len(set(labels))))

    print(group)
