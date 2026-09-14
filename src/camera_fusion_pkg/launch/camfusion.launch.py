from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='autobus/camaras',
        description='Namespace para los nodos y tópicos de cámara'
    )
    namespace = LaunchConfiguration('namespace')

    # El params.yaml del paquete C++ sobreescribe los defaults del nodo.
    # swap_default dentro del yaml puede seguir cambiándose en caliente con
    # el servicio /camera_fusion/swap_cameras o desde la GUI.
    params_file = PathJoinSubstitution([
        FindPackageShare('camera_fusion_cpp_pkg'),
        'config',
        'params.yaml'
    ])

    fusion_node = Node(
        package='camera_fusion_cpp_pkg',
        executable='camera_fusion_cpp_node',
        name='panoramic_fusion',
        namespace=namespace,
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        namespace_arg,
        fusion_node,
    ])
