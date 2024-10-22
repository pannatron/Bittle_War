from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    device1 = LaunchConfiguration('device1')
    device2 = LaunchConfiguration('device2')
    v4l2_ctl_path = '/usr/bin/v4l2-ctl'  # Specify the full path to v4l2-ctl

    return LaunchDescription([
        # Arguments for first camera
        DeclareLaunchArgument(
            'device1',
            default_value='/dev/video2',
            description='Video device 1'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='10/1',
            description='Frame rate'
        ),
        DeclareLaunchArgument(
            'width',
            default_value='1280',
            description='Frame width'
        ),
        DeclareLaunchArgument(
            'height',
            default_value='720',
            description='Frame height'
        ),
        DeclareLaunchArgument(
            'publish_frame1',
            default_value='false',
            description='Whether to publish the frame for device 1'
        ),

        # Arguments for second camera
        DeclareLaunchArgument(
            'device2',
            default_value='/dev/video0',
            description='Video device 2'
        ),
        DeclareLaunchArgument(
            'publish_frame2',
            default_value='false',
            description='Whether to publish the frame for device 2'
        ),

        # Set auto focus to off for both cameras
        ExecuteProcess(
            cmd=[f'{v4l2_ctl_path}', '-d', LaunchConfiguration('device1'), '--set-ctrl=focus_auto=0'],
            shell=True
        ),
        ExecuteProcess(
            cmd=[f'{v4l2_ctl_path}', '-d', LaunchConfiguration('device2'), '--set-ctrl=focus_auto=0'],
            shell=True
        ),

        # Node for first camera
        Node(
            package='gscam',
            executable='gscam_node',
            name='gscam_driver_v4l_1',
            namespace='v4l1',
            output='screen',
            parameters=[{
                'camera_name': 'camera1',
                'camera_info_url': 'package://gscam/examples/uncalibrated_parameters.ini',
                'gscam_config': [
                    'v4l2src device=', device1,
                    ' ! image/jpeg,width=', LaunchConfiguration('width'),
                    ',height=', LaunchConfiguration('height'),
                    ',framerate=', LaunchConfiguration('fps'),
                    ' ! jpegdec ! videoconvert ! video/x-raw,format=RGB ! queue'
                ],
                'frame_id': '/v4l_frame1',
                'sync_sink': True
            }],
            arguments=['--gst-debug=2']
        ),

        # Node for second camera
        Node(
            package='gscam',
            executable='gscam_node',
            name='gscam_driver_v4l_2',
            namespace='v4l2',
            output='screen',
            parameters=[{
                'camera_name': 'camera2',
                'camera_info_url': 'package://gscam/examples/uncalibrated_parameters.ini',
                'gscam_config': [
                    'v4l2src device=', device2,
                    ' ! image/jpeg,width=', LaunchConfiguration('width'),
                    ',height=', LaunchConfiguration('height'),
                    ',framerate=', LaunchConfiguration('fps'),
                    ' ! jpegdec ! videoconvert ! video/x-raw,format=RGB ! queue'
                ],
                'frame_id': '/v4l_frame2',
                'sync_sink': True
            }],
            arguments=['--gst-debug=2']
        ),

        # Publish frame for first camera
        Node(
            condition=IfCondition(LaunchConfiguration('publish_frame1')),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='v4l_transform_1',
            arguments=['1', '2', '3', '0', '-3.141', '0', '/world', '/v4l_frame1']
        ),

        # Publish frame for second camera
        Node(
            condition=IfCondition(LaunchConfiguration('publish_frame2')),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='v4l_transform_2',
            arguments=['1', '2', '3', '0', '-3.141', '0', '/world', '/v4l_frame2']
        )
    ])

