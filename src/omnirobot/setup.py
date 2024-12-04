from setuptools import find_packages, setup

package_name = 'omnirobot'

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
    maintainer='timoha',
    maintainer_email='timagadenov1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = omnirobot.aruco_detector:main',
            'distance_publisher = omnirobot.distance_publisher:main',
            'audio = omnirobot.robot_audio:main',
            'laser_follow = omnirobot.laser_keep_distance:main',
            'main_node = omnirobot.main_node:main',
            'motor_control = omnirobot.motor_control:main',
            'CAM = omnirobot.camera_vision_publisher:main',
            'barrel_detector = omnirobot.barrel_detector:main',
            'main_alg = omnirobot.main_alg:main',
        ],
    },
)
