from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='autobus/camaras',
        description='Namespace para los nodos y tópicos de cámara'
    )
    # mode:='sync' (default) usa el nodo con sincronizador de timestamps
    # mode:='async' usa el nodo de timer fijo sin sincronizador (más fluido)
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='async',
        description="Modo de fusión: 'sync' o 'async'"
    )

    namespace = LaunchConfiguration('namespace')
    mode      = LaunchConfiguration('mode')

    reader_node = Node(
        package='camera_fusion_pkg',
        executable='camera_reader_node',
        name='camera_reader',
        namespace=namespace,
        output='screen'
    )

    fusion_sync = Node(
        package='camera_fusion_pkg',
        executable='camera_fusion_node',
        name='panoramic_fusion',
        namespace=namespace,
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sync'"]))
    )

    fusion_async = Node(
        package='camera_fusion_pkg',
        executable='camera_fusion_async_node',
        name='panoramic_fusion',
        namespace=namespace,
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'async'"]))
    )

    return LaunchDescription([
        namespace_arg,
        mode_arg,
        reader_node,
        fusion_sync,
        fusion_async,
    ])
