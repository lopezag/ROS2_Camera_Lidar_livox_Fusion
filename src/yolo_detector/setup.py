from setuptools import find_packages, setup

package_name = 'yolo_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    	'setuptools',
    	'opencv-python',
    	'ultralytics',
    ],
    zip_safe=True,
    maintainer='lopezag',
    maintainer_email='lopezag01@gmail.com',
    description='Nodo ROS2 para correr YOLOv8 sobre el feed de la cámara',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'detector_node = yolo_detector.detector_node:main',
        ],
    },
)
