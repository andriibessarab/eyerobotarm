from setuptools import find_packages, setup

package_name = 'task_coordinator'

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
    maintainer='team',
    maintainer_email='andriibessarab@gmail.com',
    description='Orchestrates the gaze-guided pick-and-place workflow.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_coordinator_node = task_coordinator.task_coordinator_node:main',
        ],
    },
)
