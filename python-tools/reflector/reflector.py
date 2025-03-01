import json
import copy
import sys

def reflect_point_y_axis(point, ywidth):
    return {
        "x": point["x"],
        "y": ywidth - point["y"]
    }

def reflect_rotation(rotation):
    return (360 - rotation) % 360

def reflect_path(input_path, output_path, ywidth):
    with open(input_path, 'r') as file:
        data = json.load(file)

    reflected_data = copy.deepcopy(data)

    for waypoint in reflected_data["waypoints"]:
        waypoint["anchor"] = reflect_point_y_axis(waypoint["anchor"], ywidth)
        if waypoint["prevControl"]:
            waypoint["prevControl"] = reflect_point_y_axis(waypoint["prevControl"], ywidth)
        if waypoint["nextControl"]:
            waypoint["nextControl"] = reflect_point_y_axis(waypoint["nextControl"], ywidth)
        waypoint["linkedName"] = None

    for rotation_target in reflected_data["rotationTargets"]:
        rotation_target["rotation"] = reflect_rotation(rotation_target["rotation"])

    for zone in reflected_data.get("pointTowardsZones", []):
        zone["fieldPosition"] = reflect_point_y_axis(zone["fieldPosition"], ywidth)

    reflected_data["goalEndState"]["rotation"] = reflect_rotation(reflected_data["goalEndState"]["rotation"])
    reflected_data["idealStartingState"]["rotation"] = reflect_rotation(reflected_data["idealStartingState"]["rotation"])

    with open(output_path, 'w') as file:
        json.dump(reflected_data, file, indent=4)

def main():
    if len(sys.argv) < 2:
        print("Usage: python reflect_paths.py <path1> <path2> ...")
        sys.exit(1)

    ywidth = 8.05
    input_paths = sys.argv[1:]

    for input_path in input_paths:
        output_path = input_path.replace('.path', '-reflected.path')
        reflect_path(input_path, output_path, ywidth)
        print(f"Reflected: {input_path} -> {output_path}")

if __name__ == "__main__":
    main()
