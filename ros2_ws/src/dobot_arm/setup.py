from setuptools import find_packages, setup

package_name = 'dobot_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pydobot', 'pyserial'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='andriibessarab@gmail.com',
    description='ROS2 wrapper node for the Dobot Magician arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dobot_arm_node = dobot_arm.dobot_arm_node:main',
        ],
    },
)
