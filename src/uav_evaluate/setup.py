from setuptools import find_packages, setup

package_name = 'uav_evaluate'
repo_env_config = '../../config/uav_env.sh'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [repo_env_config]),
    ],
    scripts=[
        'scripts/evaluate_xai_sac',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV Maintainers',
    maintainer_email='uav@example.com',
    description='Evaluate trained XAI SAC policies on the UAV ROS interface.',
    license='MIT',
    tests_require=['pytest'],
)
