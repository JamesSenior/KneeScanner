import open3d as o3d
import numpy as np
import json
from pathlib import Path


def main():

    #Start
    print("PLY Viewer")

    

    #read in point cloud
    cloud1 = o3d.io.read_point_cloud("master.ply")
    cloud2 = o3d.io.read_point_cloud("scan.ply")
    cloud3 = o3d.io.read_point_cloud("output.ply")

    cloud1.paint_uniform_color([1, 0, 0])  # red
    cloud2.paint_uniform_color([0, 1, 0])  # green
    cloud3.paint_uniform_color([0, 0, 1])  # blue

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    vis.add_geometry(cloud1)
    vis.add_geometry(cloud2)
    vis.add_geometry(cloud3)

    vis.run()





#run main() if it is run, if it is just included by other project it iwll not imidiatly run
if __name__ == "__main__":
    main()
