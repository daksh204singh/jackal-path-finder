from setuptools import setup
from glob import glob
import os

package_name = 'jackal_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install package.xml into share/package_name
        ('share/' + package_name, ['package.xml']),
        # Install all launch files into share/package_name/launch
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/nav2_params.yaml')),
        ('share/' + package_name + '/maps', glob('maps/map_1744446424.yaml')),
        ('share/' + package_name + '/maps', glob('maps/map_1744446424.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    description='Description of the package',
    license='License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
