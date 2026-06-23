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

    reader_node = Node(
        package='camera_fusion_pkg',
        executable='camera_reader_node',
        name='camera_reader',
        namespace=namespace,
        output='screen',
        parameters=[{
            'fps':         30,
            'width':       640,    # resolución solicitada a V4L2 (puede ignorarse)
            'height':      640,    # resolución solicitada a V4L2 (puede ignorarse)
            'target_size': 640,    # resolución de salida GARANTIZADA (resize siempre aplicado)
        }]
    )

    fusion_node = Node(
        package='camera_fusion_pkg',
        executable='camera_multicams_node',
        name='multicams_fusion',
        namespace=namespace,
        output='screen',
        parameters=[{
            'canvas_w': 640,
            'canvas_h': 640,
            'cam_w':    640,
            'cam_h':    640,
            'slop_ms':  150,
        }]
    )

    return LaunchDescription([
        namespace_arg,
        reader_node,
        fusion_node,
    ])
