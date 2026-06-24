from setuptools import find_packages, setup

package_name = 'uav_control'

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
    description='Keyboard and high-level control nodes for UAV.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_cmd_vel = uav_control.keyboard_cmd_vel:main',
        ],
    },
)
