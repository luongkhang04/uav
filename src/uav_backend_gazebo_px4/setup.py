from setuptools import find_packages, setup

package_name = 'uav_backend_gazebo_px4'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khang',
    maintainer_email='khang@example.com',
    description='Gazebo PX4 backend adapter and monitor.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_offboard_adapter = uav_backend_gazebo_px4.px4_offboard_adapter:main',
            'state_monitor = uav_backend_gazebo_px4.state_monitor:main',
        ],
    },
)
