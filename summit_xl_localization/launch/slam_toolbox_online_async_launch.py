import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')

    namespace_argument = DeclareLaunchArgument(
    name='namespace',
    description='Namespace of the robot',
    default_value='robot',
    )

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [                  
                  ('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/map', 'map')
                 ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time}

    configured_params_no_ns = RewrittenYaml(
        source_file=params_file,
        root_key=namespace, #TOF: this should only happen if use_namespace=true
        param_rewrites=param_substitutions,
        convert_types=True)
    
    configured_params = ReplaceString(
            source_file=configured_params_no_ns,
            replacements={'<robot_namespace>': ('/', namespace)})

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory("summit_xl_navigation"),
                                   'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          configured_params,
        #   {'use_sim_time': use_sim_time},
        #   {'scan_topic': ['/', namespace , '/front_laser/scan']},
        #   {'odom_frame': [namespace , '/odom']},
        #   {'map_frame': [namespace , '/map']},
        #   {'base_frame': [namespace , '/base_footprint']}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=remappings)

    ld = LaunchDescription()

    ld.add_action(LogInfo(msg=['slam configured_params',configured_params]))

    ld.add_action(namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
