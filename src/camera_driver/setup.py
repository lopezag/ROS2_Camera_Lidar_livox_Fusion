from setuptools import find_packages, setup

package_name = 'camera_driver'

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
        'pyyaml',
        # cv_bridge no se instala via pip, pero queda como dependencia en package.xml
    ],
    zip_safe=True,
    maintainer='lopezag',
    maintainer_email='lopezag01@gmail.com',
    description='Driver de cámara USB con publicación de Image y CameraInfo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_driver.camera_node:main',
        ],
    },
)

