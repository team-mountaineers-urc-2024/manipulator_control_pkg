import os 
from glob import glob # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 
from setuptools import find_packages, setup # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 

package_name = 'manipulator_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))), # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='nathanpadkins@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'manipulator_motors = manipulator_control_pkg.manipulator_motors:main',
            'manipulator_motors_can_node = manipulator_control_pkg.manipulator_motors_can_node:main',
            'manipulator_motors_uart_node = manipulator_control_pkg.manipulator_motors_uart_node:main',

            'operator_controls = manipulator_control_pkg.operator_controls:main',
            'debugnode = manipulator_control_pkg.debugnode:main',

        ],
    },
)
