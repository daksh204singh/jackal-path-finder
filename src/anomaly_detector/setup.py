from setuptools import setup

package_name = 'anomaly_detector'

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
    maintainer=['Brijesh Bhayana', 'Daksh Pratap Singh', 'Tanvin Kalra'],
    maintainer_email=['bkbhayan@ncsu.edu', 'dsingh23@ncsu.edu', 'tkalra@ncsu.edu'],
    description='Object detection for red spheres',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'anomaly_detector = anomaly_detector.anomaly_detector_node:main',
        ],
    },
)
