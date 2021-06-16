from setuptools import setup

package_name = 'iti_lab7'

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
        'sub_task1=iti_lab7.sub_task1:main',
        'sub_task2=iti_lab7.sub_task2:main',
        ],
    },
)
