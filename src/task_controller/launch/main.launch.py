import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Encontra o caminho para o seu ficheiro de parâmetros
    pkg_share_dir = get_package_share_directory('task_controller')
    params_file_path = os.path.join(pkg_share_dir, 'config', 'params.yaml')

    # Ação para iniciar o seu nó de lógica de tarefas
    start_task_logic_node = Node(
        package='task_controller',
        executable='task_logic_node',
        name='task_logic_controller',
        output='screen',
        parameters=[params_file_path] # Carrega os parâmetros do ficheiro
    )

    # Cria a descrição de lançamento e adiciona a única ação
    ld = LaunchDescription()
    ld.add_action(start_task_logic_node)

    return ld