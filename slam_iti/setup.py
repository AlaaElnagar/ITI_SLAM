import os
from glob import glob

from setuptools import setup

package_name = 'slam_iti'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alaa',
    maintainer_email='alaaelngar560@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'get_odom_scan=slam_iti.get_odom_scan:main',
        'read_plan_trajectory=slam_iti.read_plan_trajectory:main',
        'odom=slam_iti.odom:main',
        'sub_n=slam_iti.sub_n:main',
        ],
    },
)
