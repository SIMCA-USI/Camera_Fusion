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

    swap_arg = DeclareLaunchArgument(
        'swap_cameras',
        default_value='true',
        description='Invierte automáticamente el orden de las cámaras si el USB las asigna al revés'
    )
    swap_cameras = LaunchConfiguration('swap_cameras')

    fusion_node = Node(
        package='camera_fusion_cpp_pkg',
        executable='camera_fusion_cpp_node',
        name='panoramic_fusion',
        namespace=namespace,
        output='screen',
        parameters=[{
            'swap_default': swap_cameras
        }]
    )

    return LaunchDescription([
        namespace_arg,
        swap_arg,
        fusion_node,
    ])
