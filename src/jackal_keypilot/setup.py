from setuptools import setup

package_name = 'jackal_keypilot'

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
    maintainer=['Tanvin Kalra', 'Brijesh Bhayana', 'Daksh Pratap Singh'],
    maintainer_email=['tkalra@ncsu.edu', 'bkbhayan@ncsu.edu', 'dsingh23@ncsu.edu'],
    description='Keyboard piloting for Jackal robot in ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keypilot = jackal_keypilot.keypilot:main',
        ],
    },
)
