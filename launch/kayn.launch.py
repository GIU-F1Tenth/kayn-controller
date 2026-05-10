from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

_ACADOS_ROOT = os.environ.get(
    'ACADOS_SOURCE_DIR',
    os.path.join(os.path.dirname(__file__), '../../../utilities/acados'),
)

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('kayn_controller'),
        'config', 'kayn_params.yaml'
    )

    env = os.environ.copy()
    acados_py = os.path.join(_ACADOS_ROOT, 'interfaces', 'acados_template')
    acados_lib = os.path.join(_ACADOS_ROOT, 'lib')
    env['PYTHONPATH'] = acados_py + ':' + env.get('PYTHONPATH', '')
    env['LD_LIBRARY_PATH'] = acados_lib + ':' + env.get('LD_LIBRARY_PATH', '')
    env['ACADOS_SOURCE_DIR'] = _ACADOS_ROOT

    return LaunchDescription([
        Node(
            package='kayn_controller',
            executable='kayn_node',
            name='kayn_controller_node',
            parameters=[config],
            output='screen',
            emulate_tty=True,
            additional_env=env,
        )
    ])
