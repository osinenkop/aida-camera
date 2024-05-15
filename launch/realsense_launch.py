#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml, subprocess


#### Find Connected RealSense Cameras ####
s = subprocess.getstatusoutput(f'rs-enumerate-devices | grep Serial ')
s = s[-1].split("\n")
serial = []
for line in s:
    if not "Asic" in line:
        num = line.split("\t")[-1]
        serial.append(num)


#### Get the Config Files ####
package_name = os.path.realpath(__file__).split("/")[-3]
config_directory = os.path.join(get_package_share_directory(package_name), 'config')
config_list = os.listdir(config_directory)
config_files = [f for f in config_list if f.startswith("realsense")]


#### Reading Config Files ####
node_list = []
for file_name in config_files:
    file_directory = os.path.join(get_package_share_directory(package_name), 'config', file_name)
    with open(file_directory, 'r') as file: data = yaml.safe_load(file)

    if data["serial"] in serial: 
            data['serial_no'] = data.pop('serial')
            name = data["camera_name"]

            position = data.pop("position").split(",")
            orientation = data.pop("orientation").split(",")

            param = [{key: value} for key, value in data.items()]
            
            node_list.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = [position[0], position[1], position[2], orientation[0], orientation[1], orientation[2], name+"_point_cloud", name+"_robot"]
            ))

            node_list.append(
                Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace="RealSense",
                name = name,
                parameters=param,
                output="screen",
                        ))

#### Launch ####
def generate_launch_description():
    return LaunchDescription(node_list)

        
    


