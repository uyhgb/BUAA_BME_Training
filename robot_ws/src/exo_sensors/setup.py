from setuptools import setup
from glob import glob
import os

package_name = 'exo_sensors'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='外骨骼传感器数据采集和处理包 - 用于步态识别的IMU数据读取',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_csv_reader = exo_sensors.imu_csv_reader:main',
            'imu_data_recorder = exo_sensors.imu_data_recorder:main',
            'imu_visualizer = exo_sensors.imu_visualizer:main',
            'foot_state_machine = exo_sensors.foot_state_machine:main',
        ],
    },
)
