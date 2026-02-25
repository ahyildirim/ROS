from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_first_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join(os.path.dirname(__file__), 'launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahyildir',
    maintainer_email='ahmetshelby@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'temp_sensor = my_first_pkg.temp_sensor:main',
            'display_node = my_first_pkg.display_node:main',
            'imu_node = my_first_pkg.imu_node:main',
            #'depth_node = my_first_pkg.depth_node:main',
            'fusion_node = my_first_pkg.fusion_node:main',
            'depth_controller = my_first_pkg.depth_controller:main',
            'motor_sim_node = my_first_pkg.motor_sim_node:main',
        ],
    },
)
