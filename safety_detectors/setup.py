from setuptools import find_packages, setup

package_name = 'safety_detectors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', [
            'safety_detectors/models/water_classifier.keras',
            'safety_detectors/models/best.pt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giselly',
    maintainer_email='gisellyareis@gmail.com',
    description='ROS 2 package for water classification and detection',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "image_controller_node = safety_detectors.image_controller_node:main",
            "relay_node = safety_detectors.relay_node:main",
            "image_publisher = safety_detectors.image_publisher:main",
            "image_subscriber = safety_detectors.image_subscriber:main",
            "water_classifier_node = safety_detectors.water_classifier_node:main",
            "water_detection_node = safety_detectors.water_detection_node:main",
            "service = safety_detectors.water_detection_service:main",
            "client = safety_detectors.water_detection_client:main",
        ],
    },
)
