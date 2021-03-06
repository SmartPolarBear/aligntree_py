import laspy
import numpy as np

import argparse

import open3d as o3d


def read_laz_point_cloud(name: str) -> np.ndarray:
    las = laspy.read(name)
    points = np.vstack((las.X, las.Y, las.Z)).transpose()
    return points


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-o', '--output',
                        help="output")

    parser.add_argument('-i', '--input',
                        help="input")

    args = parser.parse_args()

    input_file: str = str(args.input)
    print("Read {}".format(input_file))
    points = read_laz_point_cloud(input_file)

    print("Construct point cloud")
    original = o3d.geometry.PointCloud()
    original.points = o3d.utility.Vector3dVector(points)

    output_file: str = str(args.output)
    print("Write file {}".format(output_file))
    if output_file.endswith('npy'):
        np.save(output_file, points)
    else:
        o3d.io.write_point_cloud(output_file, pointcloud=original, print_progress=True)
