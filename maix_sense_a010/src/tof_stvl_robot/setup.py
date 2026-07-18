from glob import glob

from setuptools import setup

package_name = 'tof_stvl_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='ros_estudo',
    maintainer_email='ros_estudo@example.com',
    description='Production point cloud preprocessing for MaixSense A010.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pointcloud_preprocessor = '
            'tof_stvl_robot.pointcloud_preprocessor:main',
        ],
    },
)
