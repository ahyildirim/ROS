from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Terminalden alınabilecek parametreler
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency', default_value='2.0', description='Yayın frekansı (Hz)'
    )
    temp_min_arg = DeclareLaunchArgument(
        'temp_min', default_value='15.0', description='Minimum sıcaklık'
    )
    temp_max_arg = DeclareLaunchArgument(
        'temp_max', default_value='35.0', description='Maksimum sıcaklık'
    )
    max_threshold_arg = DeclareLaunchArgument(
        'max_temp_warning_threshold', default_value='28.0', description='Uyarı eşiği'
    )
    min_threshold_arg = DeclareLaunchArgument(
        'min_temp_warning_threshold', default_value='22.0', description='Uyarı eşiği'
    )

    # Node’lar
    temp_sensor = Node(
        package='my_first_pkg',
        executable='temp_sensor',
        name='temp_sensor',
        parameters=[{
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'temp_max': LaunchConfiguration('temp_max'),
            'temp_min': LaunchConfiguration('temp_min')
        }],
        output='screen'
    )

    display_node = Node(
        package='my_first_pkg',
        executable='display_node',
        name='display_node',
        parameters=[{
            'max_temp_warning_threshold': LaunchConfiguration('max_temp_warning_threshold'),
            'min_temp_warning_threshold': LaunchConfiguration('min_temp_warning_threshold')
        }],
        output='screen'
    )

    return LaunchDescription([
        publish_frequency_arg,
        temp_min_arg,
        temp_max_arg,
        min_threshold_arg,
        max_threshold_arg,
        display_node,
        temp_sensor
    ])
