from setuptools import find_packages, setup

package_name = 'gamepad'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'evdev'],
    zip_safe=True,
    maintainer='namazyake.dev',
    maintainer_email='namazyake.dev@gmail.com',
    description='ROS2 drivers for gamepad controllers such as DualSense, publishing controller states.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dualsense_node = gamepad.dualsense_node:main'
        ],
    },
)
