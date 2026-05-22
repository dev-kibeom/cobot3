from glob import glob

from setuptools import find_packages, setup

package_name = 'vision'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('vision/models/*.pt')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/docs', glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kibeom',
    maintainer_email='neopkrrl@gmail.com',
    description='ROS2 vision nodes for OBB object detection and pick/place goal generation.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'alignment_brain_node = vision.alignment_brain_node:main',
            'brain = vision.alignment_brain_node:main',
            'color_obb_eye_node = vision.color_obb_eye_node:main',
            'color_shape_obb = vision.color_shape_obb:main',
            'goal_gateway = vision.goal_gateway:main',
            'yolo11s_obb_eye_node = vision.yolo11s_obb_eye_node:main',
            'yolo = vision.yolo11s_obb_eye_node:main',
        ],
    },
)
