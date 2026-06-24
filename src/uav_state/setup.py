from setuptools import find_packages, setup

package_name = 'uav_state'

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
    description='Backend-independent UAV state monitoring tools.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_monitor = uav_state.state_monitor:main',
            'state_monitor_gui = uav_state.state_monitor_gui:main',
        ],
    },
)
