#!/usr/bin/env python3
import sys
import tempfile
import yaml


def modify_rviz_config(config_file, namespace, scan_topic, map_frame):
    config = yaml.safe_load(open(config_file, "r"))
    display = config["Visualization Manager"]["Displays"]
    global_options = config["Visualization Manager"]["Global Options"]

    rbot_model = next(filter(lambda x: x["Name"] == "RobotModel", display))
    map = next(filter(lambda x: x["Name"] == "Map", display))
    laser = next(filter(lambda x: x["Name"] == "LaserScan", display))

    rbot_model["Description Topic"][
        "Value"] = "/" + namespace + "/robot_description"
    map["Topic"]["Value"] = "/" + namespace + "/map"
    map["Update Topic"]["Value"] = "/" + namespace + "/update_map"
    laser["Topic"]["Value"] = "/" + namespace + "/" + scan_topic
    global_options["Fixed Frame"] = "/" + namespace + "/" + map_frame

    output_file = tempfile.NamedTemporaryFile("w",
                                              prefix="rviz_config_",
                                              suffix=".rviz",
                                              delete=False)
    yaml.dump(config, output_file)
    output_file.close()

    return output_file.name


if __name__ == "__main__":
    print(modify_rviz_config(sys.argv[1], sys.argv[2], sys.argv[3],
                             sys.argv[4]),
          end="")
