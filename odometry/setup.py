from setuptools import setup

package_name = 'odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lo',
    maintainer_email='alaaelngar560@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'odom_pub=odometry.odom_pub:main', 
        'publish_gps_messge=odometry.publish_gps_messge:main', 
        ],
    },
)
