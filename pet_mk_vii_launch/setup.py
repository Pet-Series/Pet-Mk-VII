from setuptools import setup
import os
from glob import glob

package_name = 'pet_mk_vii_launch'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seniorKullken',
    maintainer_email='stefan.kull@gmail.com',
    description='Launch/bringup package for Pet-Mk.VII(seven) - The Ackermann Vehicle/robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
