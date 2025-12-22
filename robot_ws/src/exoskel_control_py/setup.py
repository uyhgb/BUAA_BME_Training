from setuptools import setup
from glob import glob
import os

package_name = 'exoskel_control_py'

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
        (os.path.join('share', package_name, 'config'), glob('config/*.pkl')),
        # Launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='外骨骼控制Python功能包 - 步态识别与力矩控制',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_inference_node = exoskel_control_py.gait_inference_node:main',
            'motor_driver_node = exoskel_control_py.motor_driver_node:main',
        ],
    },
)
