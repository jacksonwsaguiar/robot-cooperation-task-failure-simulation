from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'task_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ADICIONE AS LINHAS ABAIXO
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.[pxy][yma]'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jackson', # Verifique se seu nome e email estão corretos
    maintainer_email='jackson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'task_logic_node = task_controller.task_logic_node:main',
    ],
},
)