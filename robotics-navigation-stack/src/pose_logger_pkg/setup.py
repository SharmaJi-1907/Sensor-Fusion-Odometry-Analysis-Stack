from setuptools import find_packages, setup

package_name = 'pose_logger_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pose_logger.launch.py']),
        ('share/' + package_name + '/config', ['config/pose_logger_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinav',
    maintainer_email='sharmaabhinav1907@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_logger_node = pose_logger_pkg.pose_logger_node:main',
        ],
    },
)
