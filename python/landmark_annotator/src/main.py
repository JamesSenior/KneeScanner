import open3d as o3d
import numpy as np
import json
from pathlib import Path


def main():

    #EDIT THESE
    fileName = "jamesV2.ply"
    labels = ["Big Toe", "Middle Toe", "Small Toe"]


    #Start
    print("Landmark Annotator")


    #sort file names and locations
    jsonFileName = Path(fileName).stem + ".json"

    script_directory = Path(__file__).resolve().parent.parent

    resources_directory = (script_directory/"resources").resolve()

    ply_file = resources_directory / fileName
    json_file = resources_directory / jsonFileName


    #read in point cloud
    point_cloud = o3d.io.read_point_cloud(str(ply_file))

    if not ply_file.exists():
        print(f"Could not find file: {ply_file}")
        quit()
    
    #instructions
    print(f"Loaded {len(point_cloud.points)} points")
    print(f"Annotating {len(labels)} landmarks")

    print("Hold CTRL SHIFT and select the point specified, then close the window")



    #loop:

    landmarks = []

    for label in labels:

        print()
        print(f"Select landmark: {label}")


        # Create visualizer
        visualizer = o3d.visualization.VisualizerWithEditing()

        visualizer.create_window(
            window_name=f"Select: {label}"
        )

        visualizer.add_geometry(point_cloud)


        # Open window
        visualizer.run()

        # Get selected point indices
        picked_indices = visualizer.get_picked_points()

        # Close window
        visualizer.destroy_window()

        # Make sure exactly one point was selected
        if len(picked_indices) != 1:

            print(
                f"Expected 1 point, "
                f"but selected {len(picked_indices)}"
            )

            # Try this label again
            quit()

        # Get selected index
        index = picked_indices[0]

        # Get coordinates
        point = np.asarray(point_cloud.points)[index]

        # Store landmark
        landmark = {
            "label": label,
            "index": int(index),
            "point": point.tolist()
        }

        landmarks.append(landmark)

        print(
            f"Saved {label}: "
            f"index={index}, "
            f"point={point}"
        )
    #end if loop

    #now have landmarks variable
    #print list
    print()
    print("All landmarks:")

    for landmark in landmarks:
        print(landmark)



    #convert to json
    with open(json_file, "w") as file:
        json.dump(landmarks, file, indent=4)






#run main() if it is run, if it is just included by other project it iwll not imidiatly run
if __name__ == "__main__":
    main()