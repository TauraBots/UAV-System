from setuptools import setup

package_name = 'drone_communication'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mavros.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thxssio',
    maintainer_email='thxssio@gmail.com',
    description='Drone Communication for ROS 2',
    license='BSD',
    entry_points={
        'console_scripts': [
            'arming_service = drone_communication.arming_service:main',
            'flight_mode_service = drone_communication.flight_mode_service:main',
        ],
    },
)
