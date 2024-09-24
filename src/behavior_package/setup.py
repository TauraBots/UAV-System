from setuptools import find_packages, setup

package_name = 'behavior_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thxssio',
    maintainer_email='thxssio@gmail.com',
    description='Pacote de navegação baseado em waypoints.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_node = behavior_package.navigate_node:main',  # Entry point do script de navegação
        ],
    },
)
