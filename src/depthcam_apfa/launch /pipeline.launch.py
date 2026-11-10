from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # -------- Gazebo Sim --------
    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-v4', '-r',
            'wildthumper_runway.sdf'
        ],
        output='screen'
    )

    # -------- ArduPilot SITL --------
    ardupilot = ExecuteProcess(
        cmd=[
            'sim_vehicle.py',
            '--map',
            '--console',
            '-v', 'Rover',
            '--model', 'JSON',
            '-f', 'rover-skid'
        ],
        output='screen'
    )

    # -------- MAVROS --------
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            {'fcu_url': 'udp://127.0.0.1:14550@14555'}
        ]
    )

    # -------- Gazebo ↔ ROS bridge (Depth Camera) --------
    depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/world/runway/model/wildthumper_4wd/link/depth_camera_link/sensor/rs_front/depth_image'
            '@sensor_msgs/msg/Image@gz.msgs.Image',

            '/world/runway/model/wildthumper_4wd/link/depth_camera_link/sensor/rs_front/camera_info'
            '@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ]
    )

    # -------- Your custom node --------
    apfa_node = Node(
        package='depthcam_apfa',
        executable='apfa',
        name='apfa',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        ardupilot,
        mavros,
        depth_bridge,
        apfa_node
    ])
