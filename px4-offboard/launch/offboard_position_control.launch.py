from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile


def generate_launch_description():

    # Namespace configurável (apenas para TÓPICOS, não para frames TF)
    namespace = LaunchConfiguration('namespace', default='px4_offboard')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='px4_offboard',
            description='Namespace of the nodes'
        ),
        Node(
            package='px4_offboard',
            namespace=namespace,
            executable='visualizer',
            name='visualizer',
            parameters=[
                {'namespace': namespace},
                {'use_sim_time': False}  # ajuste se não estiver em simulação
            ]
        ),
        Node(
            package='px4_offboard',
            namespace=namespace,
            executable='offboard_control',
            name='control',
            parameters=[
                {'radius': 10.0},
                {'altitude': 5.0},
                {'omega': 0.5},
                {'use_sim_time': False},  # ajuste se não estiver em simulação
                {'namespace': namespace}
            ]
        ),
        OpaqueFunction(function=launch_setup),
    ])


def patch_rviz_config(original_config_path, namespace: str):
    """
    Patch do arquivo RViz:
    - __NS__  -> '/<namespace>' em TÓPICOS
    - NUNCA colocar namespace em frames TF
    - Força Fixed Frame = odom (e Target Frame = odom se existir)
    """
    import re
    with open(original_config_path, 'r') as f:
        content = f.read()

    ns = f'/{namespace}' if namespace else ''

    # 1) Namespace só onde houver __NS__ (use __NS__/meu_topico no .rviz)
    content = content.replace('__NS__', ns)

    # 2) Garante que frames clássicos não fiquem namespaced por engano
    for frame in ['world', 'map', 'odom', 'base_link']:
        content = content.replace(f'{ns}/{frame}', frame)

    # 3) Força Fixed Frame = odom
    if 'Fixed Frame:' in content:
        content = re.sub(r'(^\s*Fixed Frame:\s*).+$', r'\1odom', content, flags=re.MULTILINE)
    else:
        # se a chave não existir, injeta dentro de Global Options
        content = re.sub(r'(^\s*Global Options:\s*\n)', r'\1  Fixed Frame: odom\n', content, flags=re.MULTILINE)

    # 4) (Opcional) força Target Frame = odom, se existir
    content = re.sub(r'(^\s*Target Frame:\s*).+$', r'\1odom', content, flags=re.MULTILINE)

    # escreve em um arquivo temporário
    tmp_rviz_config = tempfile.NamedTemporaryFile(delete=False, suffix='.rviz')
    tmp_rviz_config.write(content.encode('utf-8'))
    tmp_rviz_config.close()
    return tmp_rviz_config.name


def launch_setup(context, *args, **kwargs):
    """
    Prepara RViz com o arquivo configurado (Fixed Frame deve ser 'odom' no .rviz).
    """
    namespace = LaunchConfiguration('namespace').perform(context)
    rviz_config_path = os.path.join(
        get_package_share_directory('px4_offboard'), 'visualize.rviz'
    )
    patched_config = patch_rviz_config(rviz_config_path, namespace)

    return [
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', patched_config],
            parameters=[{'use_sim_time': True}]  # ajuste se preciso
        )
    ]

