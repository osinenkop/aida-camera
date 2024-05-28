# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

import os
import xacro, yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, conditions
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.substitutions import LaunchConfiguration, Command

import launch.actions
import launch_ros.actions

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

def to_urdf(xacro_path, urdf_path=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * urdf_path -- the path to the urdf file
    """
    # If no URDF path is given, use a temporary file
    if urdf_path is None:
        urdf_path = os.path.join(
            get_package_share_directory("azure_kinect_ros_driver"),
            "urdf",
            "azure_kinect.urdf")
    # open and process file
    doc = xacro.process_file(xacro_path)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path  # Return path to the urdf file

def launch_setup(context, params):

    print(params)
    return launch_ros.actions.Node(
        package='azure_kinect_ros_driver',
        executable='node',
        output='screen',
        namespace = "",

        parameters=[
            {"device_id": (params["index"]) + "_"},
            {'depth_enabled': True if params["depth_enabled"] == "true" else False},
            {'depth_mode': params["depth_mode"]},
            {'depth_unit': params["depth_unit"]},
            {'color_enabled': True if params["color_enabled"] == "true" else False},
            {'color_format': params["color_format"]},
            {'color_resolution': params["color_resolution"]},
            {'fps': int(params["fps"])},
            {'point_cloud': True if  params["point_cloud"] == "true" else False},
            {'rgb_point_cloud': True if  params["rgb_point_cloud"] == "true" else False},
            {'point_cloud_in_depth_frame': True if  params["point_cloud_in_depth_frame"] == "true" else False},
            {'sensor_sn': str(params["sensor_sn"]) + "_"},
            {'recording_file': params["recording_file"]},
            {'recording_loop_enabled': True if  params["recording_loop_enabled"] == "true" else False},
            {'body_tracking_enabled': True if  params["body_tracking_enabled"] == "true" else False},
            {'body_tracking_smoothing_factor': float(params["body_tracking_smoothing_factor"])},
            {'rescale_ir_to_mono8': True if params["rescale_ir_to_mono8"] == "true" else False},
            {'ir_mono8_scaling_factor': float(params["ir_mono8_scaling_factor"])},
            {'imu_rate_target': int(params["imu_rate_target"])},
            {'wired_sync_mode': int(params["wired_sync_mode"])},
            {'subordinate_delay_off_master_usec': int(params["subordinate_delay_off_master_usec"])}]),
    
        
        # parameters=[
        #     {"device_id": str(launch.substitutions.LaunchConfiguration('device_id').perform(context)) + "_"},
        #     {'depth_enabled': True if launch.substitutions.LaunchConfiguration('depth_enabled').perform(context) == "true" else False},
        #     {'depth_mode': launch.substitutions.LaunchConfiguration('depth_mode').perform(context)},
        #     {'depth_unit': launch.substitutions.LaunchConfiguration('depth_unit').perform(context)},
        #     {'color_enabled': True if launch.substitutions.LaunchConfiguration('color_enabled').perform(context) == "true" else False},
        #     {'color_format': launch.substitutions.LaunchConfiguration('color_format').perform(context)},
        #     {'color_resolution': launch.substitutions.LaunchConfiguration('color_resolution').perform(context)},
        #     {'fps': int(launch.substitutions.LaunchConfiguration('fps').perform(context))},
        #     {'point_cloud': True if  launch.substitutions.LaunchConfiguration('point_cloud').perform(context) == "true" else False},
        #     {'rgb_point_cloud': True if  launch.substitutions.LaunchConfiguration('rgb_point_cloud').perform(context) == "true" else False},
        #     {'point_cloud_in_depth_frame': True if  launch.substitutions.LaunchConfiguration('point_cloud_in_depth_frame').perform(context) == "true" else False},
        #     {'sensor_sn': str(launch.substitutions.LaunchConfiguration('sensor_sn').perform(context)) + "_"},
        #     {'recording_file': launch.substitutions.LaunchConfiguration('recording_file').perform(context)},
        #     {'recording_loop_enabled': True if  launch.substitutions.LaunchConfiguration('recording_loop_enabled').perform(context) == "true" else False},
        #     {'body_tracking_enabled': True if  launch.substitutions.LaunchConfiguration('body_tracking_enabled').perform(context) == "true" else False},
        #     {'body_tracking_smoothing_factor': float(launch.substitutions.LaunchConfiguration('body_tracking_smoothing_factor').perform(context))},
        #     {'rescale_ir_to_mono8': True if launch.substitutions.LaunchConfiguration('rescale_ir_to_mono8').perform(context) == "true" else False},
        #     {'ir_mono8_scaling_factor': float(launch.substitutions.LaunchConfiguration('ir_mono8_scaling_factor').perform(context))},
        #     {'imu_rate_target': int(launch.substitutions.LaunchConfiguration('imu_rate_target').perform(context))},
        #     {'wired_sync_mode': int(launch.substitutions.LaunchConfiguration('wired_sync_mode').perform(context))},
        #     {'subordinate_delay_off_master_usec': int(launch.substitutions.LaunchConfiguration('subordinate_delay_off_master_usec').perform(context))}]),
    

def generate_launch_description():
    # Note: tf_prefix is not supported as an argument to the xacro file for robot/joint state publishers
    # Convert xacro to urdf for robot_state_publisher and joint_state_publisher
    xacro_file = os.path.join(
            get_package_share_directory("azure_kinect_ros_driver"),
            "urdf",
            "azure_kinect.urdf.xacro")
    print("Robot description xacro_file : {}".format(xacro_file))

    urdf_path = to_urdf(xacro_file) # convert, xacro to urdf
    urdf = open(urdf_path).read()
    print("Robot description urdf_path : {}".format(urdf_path))

    # Variable used for the flag to publish a standalone azure_description instead of the default robot_description parameter
    remappings = [('robot_description', 'azure_description')]
        #### Find Connected Azure Kinect Cameras ####
    connected_devices = []
    cam_ports = os.popen("ls /dev | grep video")
    cam_ports = cam_ports.read().split('\n')[:-1]
    for port in cam_ports:
        info = os.popen(f"udevadm info --query=all /dev/{port} | grep 'E: ID_SERIAL=Microsoft_Azure_Kinect_4K_Camera\|ID_V4L_CAPABILITIES'")
        info = info.read().split('\n')[:-1]
        if len(info) == 2:
            cam_capability = info[0]
            cam_vendor = info[1]
            cam_serial = cam_vendor.split("_")[-1]
            
            if ("Azure_Kinect_" in cam_vendor) and ("capture" in cam_capability):
                connected_devices.append(cam_serial)


    print("SINA", connected_devices)
    #### Get the Config Files ####
    package_name = os.path.realpath(__file__).split("/")[-3]
    config_directory = os.path.join(get_package_share_directory(package_name), 'config')
    config_list = os.listdir(config_directory)
    config_files = [f for f in config_list if f.startswith("azure_")]

    print("SINA", config_files)
    #### Reading Config Files ####
    node_list = []
    for file_name in config_files:
        file_directory = os.path.join(get_package_share_directory(package_name), 'config', file_name)
        with open(file_directory, 'r') as file: data = yaml.safe_load(file)

        serial = data["sensor_sn"]
        serial_number = f"{serial}"

        index = data["index"]
        if serial_number in connected_devices:
            # DeclareLaunchArgument(
            #         'device_id',
            #         default_value=index,
            #         description="Device ID"),

            # DeclareLaunchArgument(
            #     'depth_enabled',
            #     default_value=data["depth_enabled"],
            #     description="Enable or disable the depth camera"),
            # DeclareLaunchArgument(
            #     'depth_mode',
            #     default_value=data["depth_mode"],
            #     description="Set the depth camera mode, which affects FOV, depth range, and camera resolution. See Azure Kinect documentation for full details. Valid options: NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_UNBINNED, WFOV_2X2BINNED, and PASSIVE_IR"),
            # DeclareLaunchArgument(
            #     'depth_unit',
            #     default_value=data["depth_unit"],
            #     description='Depth distance units. Options are: "32FC1" (32 bit float metre) or "16UC1" (16 bit integer millimetre)'),
            # DeclareLaunchArgument(
            #     'color_enabled',
            #     default_value=data["color_enabled"],
            #     description="Enable or disable the color camera"),
            # DeclareLaunchArgument(
            #     'color_format',
            #     default_value=data["color_format"],
            #     description="The format of RGB camera. Valid options: bgra, jpeg"),
            # DeclareLaunchArgument(
            #     'color_resolution',
            #     default_value=data["color_resolution"],
            #     description="Resolution at which to run the color camera. Valid options: 720P, 1080P, 1440P, 1536P, 2160P, 3072P"),
            # DeclareLaunchArgument(
            #     'fps',
            #     default_value=data["fps"],
            #     description="FPS to run both cameras at. Valid options are 5, 15, and 30"),
            # DeclareLaunchArgument(
            #     'point_cloud',
            #     default_value=data["point_cloud"],
            #     description="Generate a point cloud from depth data. Requires depth_enabled"),
            # DeclareLaunchArgument(
            #     'rgb_point_cloud',
            #     default_value=data['rgb_point_cloud'],
            #     description="Colorize the point cloud using the RBG camera. Requires color_enabled and depth_enabled"),
            # DeclareLaunchArgument(
            #     'point_cloud_in_depth_frame',
            #     default_value=data["point_cloud_in_depth_frame"],
            #     description="Whether the RGB pointcloud is rendered in the depth frame (true) or RGB frame (false). Will either match the resolution of the depth camera (true) or the RGB camera (false)."),
            # DeclareLaunchArgument(
            #     'sensor_sn',
            #     default_value=data["sensor_sn"],
            #     description="Sensor serial number. If none provided, the first sensor will be selected"),
            # DeclareLaunchArgument(
            #     'recording_file',
            #     default_value=data["recording_file"],
            #     description="Absolute path to a mkv recording file which will be used with the playback api instead of opening a device"),
            # DeclareLaunchArgument(
            #     'recording_loop_enabled',
            #     default_value=data["recording_loop_enabled"],
            #     description="If set to true the recording file will rewind the beginning once end of file is reached"),
            # DeclareLaunchArgument(
            #     'body_tracking_enabled',
            #     default_value=data["body_tracking_enabled"],
            #     description="If set to true the joint positions will be published as marker arrays"),
            # DeclareLaunchArgument(
            #     'body_tracking_smoothing_factor',
            #     default_value=data["body_tracking_smoothing_factor"],
            #     description="Set between 0 for no smoothing and 1 for full smoothing"),
            # DeclareLaunchArgument(
            #     'rescale_ir_to_mono8',
            #     default_value=data["rescale_ir_to_mono8"],
            #     description="Whether to rescale the IR image to an 8-bit monochrome image for visualization and further processing. A scaling factor (ir_mono8_scaling_factor) is applied."),
            # DeclareLaunchArgument(
            #     'ir_mono8_scaling_factor',
            #     default_value=data["ir_mono8_scaling_factor"],
            #     description="Scaling factor to apply when converting IR to mono8 (see rescale_ir_to_mono8). If using illumination, use the value 0.5-1. If using passive IR, use 10."),
            # DeclareLaunchArgument(
            #     'imu_rate_target',
            #     default_value=data["imu_rate_target"],
            #     description="Desired output rate of IMU messages. Set to 0 (default) for full rate (1.6 kHz)."),
            # DeclareLaunchArgument(
            #     'wired_sync_mode',
            #     default_value=data["wired_sync_mode"],
            #     description="Wired sync mode. 0: OFF, 1: MASTER, 2: SUBORDINATE."),
            # DeclareLaunchArgument(
            #     'subordinate_delay_off_master_usec',
            #     default_value=data["subordinate_delay_off_master_usec"],
            #     description="Delay subordinate camera off master camera by specified amount in usec."),
            nd = OpaqueFunction(function=launch_setup, kwargs={'params': data})
            node_list += [
                nd
            ]

    return LaunchDescription([
        DeclareLaunchArgument(
        'overwrite_robot_description',
        default_value="true" ,
        description="Flag to publish a standalone azure_description instead of the default robot_description parameter."),
        DeclareLaunchArgument( # Not a parameter of the node, rather a launch file parameter
                    'required',
                    default_value="false",
                    description="Argument which specified if the entire launch file should terminate if the node dies"),

        *node_list,
    
    # If flag overwrite_robot_description is set:
    launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters = [{'robot_description' : urdf}],
        condition=conditions.IfCondition(launch.substitutions.LaunchConfiguration("overwrite_robot_description"))),
    launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_path],
        condition=conditions.IfCondition(launch.substitutions.LaunchConfiguration("overwrite_robot_description"))),
    # If flag overwrite_robot_description is not set:
    launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters = [{'robot_description' : urdf}],
        remappings=remappings,
        condition=conditions.UnlessCondition(launch.substitutions.LaunchConfiguration("overwrite_robot_description"))),
    launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_path],
        remappings=remappings,
        condition=conditions.UnlessCondition(launch.substitutions.LaunchConfiguration("overwrite_robot_description"))),
    ])