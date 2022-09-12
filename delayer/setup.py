#!/home/orch/anaconda3/envs/teleperation/bin/python3

from setuptools import setup
import os
from glob import glob

package_name = 'delayer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch and configuration files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'configs'), glob('configs/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orch',
    maintainer_email='xz20@ic.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # determine the executable function for the node
            'delayer = delayer.delayer:main',
            'delay_recorder = delayer.delay_recorder:main',
            'delay_predictor = delayer.delay_predictor:main',
            'forwarder = delayer.forwarder:main',
        ],
    },
)
