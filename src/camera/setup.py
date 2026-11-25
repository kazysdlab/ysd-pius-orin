from setuptools import find_packages, setup

package_name = 'camera'

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
    maintainer='namazyake.dev',
    maintainer_email='namazyake.dev@gmail.com',
    description='ROS2 drivers for CSI and USB cameras, publishing sensor_msgs/Image.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'csi_camera_node = camera.csi_camera_node:main'
        ],
    },
)
