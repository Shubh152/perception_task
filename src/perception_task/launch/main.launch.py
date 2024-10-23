import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='perception_task').find('perception_task')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/perception.rviz')

    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([pkg_share, 'models'])
    gz_world_path = PathJoinSubstitution([gz_model_path, 'shapes.sdf'])

    broadcast_tf_static_reference = launch_ros.actions.Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       name='static_transform_publisher',
       output='screen',
       arguments=["0", "0", "-1.15", "0", "0", "0", "depth_camera/depth_camera/depth_camera", "depth_camera/reference_frame/"]
    )


    ros_gz_bridge_pc = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"],
    )

    ros_gz_bridge_image = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image"],
    )

    ros_gz_bridge_depth = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image"],
    )

    ros_gz_bridge_info = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"],
    )

    depth_node = launch_ros.actions.Node(
        package='perception_task',
        executable='depth_node',
        output='screen'        
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig'), "--ros-args", "--remap","use_sim_time:=true"],
    )

    message_transform = launch_ros.actions.Node(
        package='message_tf_frame_transformer',
        executable='message_tf_frame_transformer',
        output='screen',
        arguments=["--ros-args", "-r","~/input:=/depth_camera/points","-r","~/transformed:=/reference/points","-p","_target_frame_id:=/depth_camera/reference"],
    )

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                    description='Absolute path to rviz config file'),

        launch.actions.DeclareLaunchArgument(
            'world',
            default_value=gz_world_path,
            description='World to load into Gazebo'
        ),
        launch.actions.SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        launch.actions.IncludeLaunchDescription( 
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': LaunchConfiguration('world'),
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        ros_gz_bridge_image,
        ros_gz_bridge_pc,
        ros_gz_bridge_depth,
        ros_gz_bridge_info,
        depth_node,
        rviz_node,
        broadcast_tf_static_reference,
    ])
