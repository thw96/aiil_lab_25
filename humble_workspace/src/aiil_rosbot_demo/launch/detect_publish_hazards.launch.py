from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    image_topic = '/oak/rgb/image_raw/compressed'
    image_topic_repeat = image_topic + '/repeat'
    use_compressed = 'true'
    
    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),

        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('image_topic', default_value=image_topic),
        DeclareLaunchArgument('image_topic_repeat', default_value=image_topic_repeat),
        DeclareLaunchArgument('use_compressed', default_value=use_compressed),

        DeclareLaunchArgument('objects_path',
            default_value=['/objects'],
            description='Directory of object templates'),
        
        DeclareLaunchArgument('settings_path',
            default_value='~/.ros/find_object_2d.ini'),

        DeclareLaunchArgument(
                                'config_file',
                                default_value=PathJoinSubstitution([
                                    FindPackageShare('aiil_rosbot_demo'),
                                    'config',
                                    'hazard_config.yaml'
                                ])
                            ),
    
        # 1. Find Object 2D Node
        Node(
            package='find_object_2d',
            executable='find_object_2d',
            output='screen',
            parameters=[{
                'subscribe_depth': False,
                'gui': LaunchConfiguration('gui'),
                'objects_path': LaunchConfiguration('objects_path'),
                'settings_path': LaunchConfiguration('settings_path')
            }],
            remappings=[
                ('image', LaunchConfiguration('image_topic_repeat')),
            ]
        ),

        # 2. Image QoS Repeater
        Node(
            package='aiil_rosbot_demo',
            executable='best_effort_repeater',
            name='best_effort_repeater',
            output='screen',
            parameters=[
                {'sub_topic_name': LaunchConfiguration('image_topic')},
                {'repeat_topic_name': LaunchConfiguration('image_topic_repeat')},
                {'use_compressed': LaunchConfiguration('use_compressed')},
            ]
        ),

        # 3. Your custom hazard publishing node
        Node(
            package='aiil_rosbot_demo',
            executable='publish_hazard',
            name='publish_hazard_test',
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
        ),
        Node(
            package='aiil_rosbot_demo',
            executable='start_on_marker',
            name='start_on_marker',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),

    ])