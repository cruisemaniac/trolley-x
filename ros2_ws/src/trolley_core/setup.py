from setuptools import find_packages, setup

package_name = 'trolley_core'

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
    maintainer='navigator',
    maintainer_email='navigator@todo.todo',
    description='Trolley-X core nodes: Arduino base controller, teleop, and LiDAR safety.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cardputer_teleop = trolley_core.cardputer_teleop:main',
            'arduino_base = trolley_core.arduino_base_controller:main',
            'safety_braking = trolley_core.safety_braking:main',
        ],
    },
)
