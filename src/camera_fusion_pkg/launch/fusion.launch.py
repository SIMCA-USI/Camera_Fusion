from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declarar el argumento del namespace para poder modificarlo desde consola
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='autobus/camaras',
        description='Namespace para los nodos y tópicos de cámara'
    )

    namespace = LaunchConfiguration('namespace')

    # Nodo de lectura de cámaras
    reader_node = Node(
        package='camera_fusion_pkg',
        executable='camera_reader_node',
        name='camera_reader',
        namespace=namespace,
        output='screen'
    )

    # Nodo de fusión panorámica
    fusion_node = Node(
        package='camera_fusion_pkg',
        executable='camera_fusion_node',
        name='panoramic_fusion',
        namespace=namespace,
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        reader_node,
        fusion_node
    ])
