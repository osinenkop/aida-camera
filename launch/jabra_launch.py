#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

cam_name = "jabra"

#### Find Connected Jabra Cameras ####
connected_devices = []
cam_ports = os.popen("ls /dev | grep video")
cam_ports = cam_ports.read().split('\n')[:-1]
port_map = {}
for port in cam_ports:
    info = os.popen(f"udevadm info --query=all /dev/{port} | grep 'DEVNAME\|ID_V4L_CAPABILITIES\|ID_SERIAL_SHORT\|ID_V4L_PRODUCT'")
    info = info.read().split('\n')[:-1]
    cam_port_name = info[0].split("/")[-1]
    cam_vendor = info[1].split("=")[-1]
    cam_capability = info[2].split("=")[-1]
    cam_serial = info[3].split("=")[-1]

    if ("Jabra" in cam_vendor) and ("capture" in cam_capability):
        port_map[cam_serial] = "/dev/" + cam_port_name


#### Get the Config Files ####
package_name = os.path.realpath(__file__).split("/")[-3]
config_directory = os.path.join(get_package_share_directory(package_name), 'config')
config_list = os.listdir(config_directory)
config_files = [f for f in config_list if f.startswith("jabra")]


#### Reading Config Files ####
node_list = []
for file_name in config_files:
    file_directory = os.path.join(get_package_share_directory(package_name), 'config', file_name)
    with open(file_directory, 'r') as file: data = yaml.safe_load(file)

    if data["serial"] in port_map.keys():
        data["port_number"] = port_map[data["serial"]]
        data["serial"] += ","

        index = data["index"]
        name = f"Jabra_Camera_{index}"

        position = data.pop("position").split(",")
        orientation = data.pop("orientation").split(",")
        
        node_list.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = [position[0], position[1], position[2], orientation[0], orientation[1], orientation[2], name, name + "_robot"]
        ))

        param = [{key: value} for key, value in data.items()]

        node_list.append(Node(package="jabra", executable="jabra_camera", name="Camera", namespace = "Jabra", parameters=param, output='screen'))


#### Launch ####
def generate_launch_description():
    return LaunchDescription(node_list)

        
    


