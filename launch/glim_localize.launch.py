import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the share directory
    glim_ros_share_dir = get_package_share_directory('glim_ros')

    # Declare launch arguments
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='',
        description='Path to the GLIM map directory to load on startup.'
    )
    initial_pose_x_arg = DeclareLaunchArgument('initial_pose_x', default_value='0.0', description='Initial pose X position')
    initial_pose_y_arg = DeclareLaunchArgument('initial_pose_y', default_value='0.0', description='Initial pose Y position')
    initial_pose_z_arg = DeclareLaunchArgument('initial_pose_z', default_value='0.0', description='Initial pose Z position')
    initial_pose_qx_arg = DeclareLaunchArgument('initial_pose_qx', default_value='0.0', description='Initial pose X orientation (quaternion)')
    initial_pose_qy_arg = DeclareLaunchArgument('initial_pose_qy', default_value='0.0', description='Initial pose Y orientation (quaternion)')
    initial_pose_qz_arg = DeclareLaunchArgument('initial_pose_qz', default_value='0.0', description='Initial pose Z orientation (quaternion)')
    initial_pose_qw_arg = DeclareLaunchArgument('initial_pose_qw', default_value='1.0', description='Initial pose W orientation (quaternion)')

    # Config file path - assuming it's often standard but could be made an argument
    config_file_path = os.path.join(glim_ros_share_dir, 'config', 'config_ros.json') # Default config, make this an arg if necessary

    # Glim ROS node
    glim_ros_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        output='screen',
        parameters=[
            {'config_path': config_file_path}, # Make sure your node uses this to find config_ros.json etc.
            # Parameters for initial pose
            {'initial_pose.position.x': LaunchConfiguration('initial_pose_x')},
            {'initial_pose.position.y': LaunchConfiguration('initial_pose_y')},
            {'initial_pose.position.z': LaunchConfiguration('initial_pose_z')},
            {'initial_pose.orientation.x': LaunchConfiguration('initial_pose_qx')},
            {'initial_pose.orientation.y': LaunchConfiguration('initial_pose_qy')},
            {'initial_pose.orientation.z': LaunchConfiguration('initial_pose_qz')},
            {'initial_pose.orientation.w': LaunchConfiguration('initial_pose_qw')},
            # New parameter for map loading on start
            {'map_load_path_on_start': LaunchConfiguration('map_path')}
        ],
        # Add remappings if necessary, e.g., for IMU, Lidar topics
        # remappings=[
        #     ('/imu/data', '/your_imu_topic'),
        #     ('/lidar/points', '/your_lidar_topic')
        # ]
    )

    return LaunchDescription([
        map_path_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_z_arg,
        initial_pose_qx_arg,
        initial_pose_qy_arg,
        initial_pose_qz_arg,
        initial_pose_qw_arg,
        glim_ros_node
    ])
