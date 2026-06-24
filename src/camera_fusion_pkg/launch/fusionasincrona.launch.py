from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='autobus/camaras',
        description='Namespace para los nodos y tópicos de cámara'
    )
    namespace = LaunchConfiguration('namespace')

    fusion_node = Node(
        package='camera_fusion_cpp_pkg',
        executable='camera_fusion_cpp_node',
        name='panoramic_fusion',
        namespace=namespace,
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        fusion_node,
    ])
