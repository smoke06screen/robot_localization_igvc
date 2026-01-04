from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navsat_setup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shubh',
    maintainer_email='shubh06kesar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_odom = navsat_setup.wheel_odom:main',
            'fake_wheel_odom = navsat_setup.fake_wheel_odom:main',
        ],
    },
)
