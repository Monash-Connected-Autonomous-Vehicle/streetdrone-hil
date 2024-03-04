from setuptools import setup
import os
from glob import glob

package_name = 'demo'

setup(
    name=package_name,
    version='0.9.4',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
				(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.json')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Dylan Gonzalez',
    author_email='dylcg10@gmail.com',
    maintainer='Thai Nguyen',
    maintainer_email='minh.nguyen6@monash.edu',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='CARLA HIL Demo package for the Monash CIV4100 unit',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ego_vehicle_control = demo.ego_vehicle_control:main',
        ],
    },
)
