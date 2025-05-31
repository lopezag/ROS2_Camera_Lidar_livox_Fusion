from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pipeline_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Registra el paquete en ament_index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Instala package.xml
        ('share/' + package_name, ['package.xml']),
        # Instala todos los launch files en share/<pkg>/launch
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='lopezag',
    maintainer_email='lopezag01@gmail.com',
    description='Launch de todo el pipeline LIDAR+CAM+IMU',
    license='Apache-2.0',
    entry_points={
        # Entry point para ros2 launch
        'launch': [
            'pipeline = pipeline_launch.pipeline_launch:generate_launch_description'
        ],
    },
)

