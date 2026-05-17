from setuptools import find_packages, setup
from glob import glob

package_name = 'aurafarm_twin'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index marker — required for ROS 2 to discover the package.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files so `ros2 launch aurafarm_twin <file>` works.
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mert Yildiz',
    maintainer_email='mert@example.com',
    description='Digital twin mediator for TurtleBot3 Burger (ROS 2 Jazzy).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Each node is a console_script so ros2 run can find it.
            'mediator = aurafarm_twin.mediator_node:main',
            'state_sync = aurafarm_twin.state_sync_node:main',
            'obstacle_monitor = aurafarm_twin.obstacle_monitor_node:main',
        ],
    },
)
