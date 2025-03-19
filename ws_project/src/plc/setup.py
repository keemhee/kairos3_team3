from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'plc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'action'), glob('src/action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kjs',
    maintainer_email='kjs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'plc_ros2 = plc.plc_test:main',
        'plc_ros22 = plc.plc_test2:main',
        'new_plc = plc.new_plc:main',
        ],
    },
)
