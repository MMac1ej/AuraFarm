from setuptools import find_packages, setup

package_name = 'aurafarm_navigation_dt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav_to_crop = aurafarm_navigation_dt.nav_to_crop:main',
            'robot_dt_bridge = aurafarm_navigation_dt.robot_dt_bridge:main',
            'lidar_real = aurafarm_navigation_dt.lidar_real_node:main',
            'lidar_sim = aurafarm_navigation_dt.lidar_sim_node:main',
            'lidar_merger = aurafarm_navigation_dt.lidar_merger_node:main',
            'sim_topic_relay = aurafarm_navigation_dt.sim_topic_relay_node:main',
        ],
    },
)
