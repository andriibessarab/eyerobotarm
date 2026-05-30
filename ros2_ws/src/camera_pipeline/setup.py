from setuptools import find_packages, setup

package_name = 'camera_pipeline'

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
    description='Camera nodes and object detection for gaze-guided pick-and-place.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'workspace_camera_node = camera_pipeline.workspace_camera_node:main',
            'gaze_camera_node = camera_pipeline.gaze_camera_node:main',
            'object_detection_node = camera_pipeline.object_detection_node:main',
        ],
    },
)
