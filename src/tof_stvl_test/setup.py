from glob import glob
from setuptools import setup

package_name = 'tof_stvl_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_estudo',
    maintainer_email='ros_estudo@example.com',
    description='Isolated STVL local costmap test with fake ToF obstacle cloud.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_obstacle_cloud = tof_stvl_test.fake_obstacle_cloud:main',
        ],
    },
)
