from setuptools import setup

package_name = 'teleop_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thxssio',
    maintainer_email='thxssio@gmail.com',
    description='Teleoperation package for controlling the drone via keyboard.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = teleop_package.teleop_node:main'
        ],
    },
)
