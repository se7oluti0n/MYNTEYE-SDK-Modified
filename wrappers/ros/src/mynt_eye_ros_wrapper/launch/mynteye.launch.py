from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown, GroupAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
  rviz = LaunchConfiguration('rviz')
  rviz_arg = DeclareLaunchArgument('rviz', default_value = 'False')
  main_config = os.path.join(
        get_package_share_directory('mynt_eye_ros_wrapper'), 'config', 'mynteye.yaml')
  standard_config = os.path.join(
        get_package_share_directory('mynt_eye_ros_wrapper'), 'config', 'device', 'standard.yaml')
  standard2_config = os.path.join(
        get_package_share_directory('mynt_eye_ros_wrapper'), 'config', 'device', 'standard2.yaml')
  process_config = os.path.join(
        get_package_share_directory('mynt_eye_ros_wrapper'), 'config', 'process', 'process_config.yaml')
  mesh_config = os.path.join(
        get_package_share_directory('mynt_eye_ros_wrapper'), 'config', 'mesh', 'mesh.yaml')

 
  mynteye_wrapper_node = Node(
    package='mynt_eye_ros_wrapper',
    executable='mynteye_wrapper_node',
    name='mynteye_wrapper_node',
    output='screen',
    parameters=[
      main_config, standard_config, standard2_config, process_config, mesh_config ]

  )

  rviz_node = GroupAction(
        condition=IfCondition(rviz),
        actions = [

            Node(
                package = 'rviz2',
                executable = 'rviz2',
                on_exit = Shutdown(),
                arguments = ['-d', FindPackageShare('mynt_eye_ros_wrapper').find('mynt_eye_ros_wrapper') + '/rviz/mynteye.rviz'],
            )
        ]
    )

  return LaunchDescription([
    rviz_arg,
    mynteye_wrapper_node,
    rviz_node
  ])
