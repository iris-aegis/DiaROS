### setup.py ###
import os
from glob import glob
from setuptools import setup

package_name = 'diaros_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='81802',
    maintainer_email='81802@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'pub = diaros_package.publisher:main',
            'relay = diaros_package.listener:main',
            'sub = diaros_package.testModule:main',
            'ros2_speech_input = diaros_package.ros2_speech_input:main',
            'ros2_acoustic_analysis = diaros_package.ros2_acoustic_analysis:main',
            'ros2_automatic_speech_recognition = diaros_package.ros2_automatic_speech_recognition:main',
            'ros2_natural_language_understanding = diaros_package.ros2_natural_language_understanding:main',
            'ros2_dialog_management = diaros_package.ros2_dialog_management:main',
            'ros2_natural_language_generation = diaros_package.ros2_natural_language_generation:main',
            'ros2_speech_synthesis = diaros_package.ros2_speech_synthesis:main',
            'ros2_speech_output = diaros_package.ros2_speech_output:main',
            'dr = diaros_package.ros2_dr:main',
            # 'mm = diaros_package.ros2_mm:main',
            'ros2_turn_taking = diaros_package.ros2_turn_taking:main',
            'ros2_back_channel = diaros_package.ros2_back_channel:main',
        ],
    },
)
