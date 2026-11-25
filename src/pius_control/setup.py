from setuptools import find_packages, setup

package_name = 'pius_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['dashboard/*.html']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pius_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antigravity AI',
    maintainer_email='antigravity@gemini.local',
    description='PIUS Autonomous Driving System Nodes: GP, P_GP, Inf, MUX, Save, Sender',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dummy_camera_node = pius_control.dummy_camera_node:main',
            'dummy_joy_node = pius_control.dummy_joy_node:main',
            'parser_node = pius_control.parser_node:main',
            'mux_node = pius_control.mux_node:main',
            'save_node = pius_control.save_node:main',
            'inference_node = pius_control.inference_node:main',
            'sender_node = pius_control.sender_node:main',
            'health_monitor_node = pius_control.health_monitor_node:main',
            'dashboard_node = pius_control.dashboard_node:main',
            'imu_node = pius_control.imu_node:main',
            'wheel_speed_node = pius_control.wheel_speed_node:main',
            'waypoint_controller_node = pius_control.waypoint_controller_node:main',
        ],
    },
)
