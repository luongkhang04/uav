from setuptools import find_packages, setup
from glob import glob

package_name = 'uav_bringup'
repo_env_config = '../../config/uav_env.sh'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', [repo_env_config]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV Maintainers',
    maintainer_email='uav@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
