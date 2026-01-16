from setuptools import find_packages, setup

package_name = 'solo_motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/solo_motors.launch.py']),
    ],
    install_requires=['setuptools', 'SoloPy'],
    zip_safe=True,
    maintainer='kunal',
    maintainer_email='devkunalmod@gamil.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'right_motor_node = solo_motor_controller.motor_controller_node:main_right',
            'left_motor_node = solo_motor_controller.motor_controller_node:main_left',
        ],
    },
)
