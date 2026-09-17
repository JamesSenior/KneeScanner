import json
from pathlib import Path

import numpy as np
import open3d as o3d


def main():
    print("PLY Viewer")

    # Resources are expected to be in the same folder as this script.
    folder = Path(__file__).resolve().parent
    ply_path = folder / "LeftLeg_james.ply"
    json_path = folder / "LeftLeg_james.json"

    # Load scan
    cloud = o3d.io.read_point_cloud(str(ply_path))

    if cloud.is_empty():
        raise RuntimeError(f"Could not read point cloud: {ply_path}")

    cloud.paint_uniform_color([0.7, 0.7, 0.7])  # grey scan

    # Load landmarks
    with open(json_path, "r", encoding="utf-8") as file:
        landmarks = json.load(file)

    # Calculate marker size from scan dimensions
    scan_size = np.linalg.norm(cloud.get_max_bound() - cloud.get_min_bound())
    marker_radius = scan_size * 0.012

    geometries = [cloud]

    print("\nLandmarks:")
    for number, landmark in enumerate(landmarks, start=1):
        label = landmark["label"]
        point = np.array(landmark["point"])

        # Create a red sphere at the landmark location
        marker = o3d.geometry.TriangleMesh.create_sphere(
            radius=marker_radius,
            resolution=20,
        )
        marker.translate(point)
        marker.paint_uniform_color([1, 0, 0])  # red
        marker.compute_vertex_normals()

        geometries.append(marker)

        print(f"{number}. {label}: {point}")

    print("\nClose the viewer window to exit.")

    o3d.visualization.draw_geometries(
        geometries,
        window_name="Left Leg Scan - Landmarks",
        width=1280,
        height=800,
    )


if __name__ == "__main__":
    main()
