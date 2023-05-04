from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown


def generate_launch_description():
  mynt_eye_arg = DeclareLaunchArgument('mynteye', default_value='mynteye')
  is_multiple_arg = DeclareLaunchArgument('is_multiple', default_value='False')
  serial_number_arg = DeclareLaunchArgument('serial_number', default_value='')
  depth_type_arg = DeclareLaunchArgument('depth_type', default_value='0')
  left_topic_arg = DeclareLaunchArgument('left_topic', default_value='left/image_raw')
  right_topic_arg = DeclareLaunchArgument('right_topic', default_value='right/image_raw')
  





  tvc_loc_tools = Node(
    package='tvc_loc_tools',
    executable='tvc_loc_tools',
    name='tvc_loc_tools',
    output='screen',
    parameters=[
      {'use_sim_time': LaunchConfiguration('use_sim_time')}]

  )

  return LaunchDescription([
    use_sim_time_arg,
    tvc_loc_tools
  ])
