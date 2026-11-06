# SPDX-License-Identifier: Apache-2.0
from glob import glob

from setuptools import setup

package_name = 'kestrel_capture'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kestrel SLAM maintainers',
    maintainer_email='sh@co.bot',
    description='Rosbag capture with live data-quality monitoring for the Orbbec Gemini 335Lg.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_monitor = kestrel_capture.monitor:main',
        ],
    },
)
