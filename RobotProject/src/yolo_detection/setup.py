from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'yolo_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cc',
    maintainer_email='cc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "yolo_detect_2d=yolo_detection.yolo_detect_2d:main",
            'navigating=yolo_detection.navigating:main'
        ],
    },
)
