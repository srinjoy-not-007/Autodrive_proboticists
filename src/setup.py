import os
from glob import glob

from setuptools import setup

package_name = 'pit_crew'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srinjoy',
    maintainer_email='t-srinjoy@saturnlabs.ai',
    description='Racing stack for RoboRacer Sim Racing League @ IROS 2026',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_the_gap = pit_crew.follow_the_gap:main',
            'calibrator = pit_crew.calibrator:main',
            'odometry = pit_crew.odometry:main',
            'localization = pit_crew.localization:main',
            'pure_pursuit = pit_crew.pure_pursuit:main',
            'arbiter = pit_crew.arbiter:main',
            'stanley = pit_crew.stanley:main',
            'mpc = pit_crew.mpc:main',
            'mpc2 = pit_crew.mpc2:main',
            'skidpad = pit_crew.skidpad:main',
        ],
    },
)
