from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'smartphone_inclinometer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    #install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david.pereira.002@student.uni.lu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "smartphone_inclinometer = smartphone_inclinometer.smartphone_inclinometer_node:main"
        ],
    },
)
