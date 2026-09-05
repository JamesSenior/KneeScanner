import open3d as o3d
import numpy as np
import json
from pathlib import Path


def main():

    #Start
    print("PLY Viewer")


#read in point cloud
    ###cloud1 = o3d.io.read_point_cloud("master.ply")
    ###cloud2 = o3d.io.read_point_cloud("scan.ply")
    ###cloud3 = o3d.io.read_point_cloud("output.ply")

    ###cloud1.paint_uniform_color([1, 0, 0])  # red
    ###cloud2.paint_uniform_color([0, 1, 0])  # green
    ###cloud3.paint_uniform_color([0, 0, 1])  # blue

    ###vis = o3d.visualization.Visualizer()
    ###vis.create_window()

    ###vis.add_geometry(cloud1)
    ###vis.add_geometry(cloud2)
    ###vis.add_geometry(cloud3)

    ###vis.run()
    

    #read in point cloud
    cloud1 = o3d.io.read_point_cloud("LeftLegMaster.ply")
    cloud2 = o3d.io.read_point_cloud("LeftLegScan.ply")

    cloud3 = o3d.io.read_point_cloud("RightLegMaster.ply")
    cloud4 = o3d.io.read_point_cloud("RightLegScan.ply")

    cloud5 = o3d.io.read_point_cloud("KneelingMaster.ply")
    cloud6 = o3d.io.read_point_cloud("KneelingScan.ply")
    

    cloud1.paint_uniform_color([0, 1, 0])  # green
    cloud2.paint_uniform_color([0, 0, 1])  # blue
    cloud3.paint_uniform_color([0, 1, 0])  # green
    cloud4.paint_uniform_color([0, 0, 1])  # blue
    cloud5.paint_uniform_color([0, 1, 0])  # green
    cloud6.paint_uniform_color([0, 0, 1])  # blue

#left leg
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    vis.add_geometry(cloud1)
    vis.add_geometry(cloud2)

    vis.run()
    vis.destroy_window()

#right leg
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    vis.add_geometry(cloud3)
    vis.add_geometry(cloud4)

    vis.run()
    vis.destroy_window()

#kneeling
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    vis.add_geometry(cloud5)
    vis.add_geometry(cloud6)

    vis.run()
    vis.destroy_window()





#run main() if it is run, if it is just included by other project it iwll not imidiatly run
if __name__ == "__main__":
    main()
