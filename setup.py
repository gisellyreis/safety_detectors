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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giselly',
    maintainer_email='gisellyareis@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "image_controller_node = safety_detectors.image_controller_node:main",
            "relay_node = safety_detectors.relay_node:main",
        ],
    },
)
