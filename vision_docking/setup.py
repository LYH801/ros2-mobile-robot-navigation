from setuptools import find_packages, setup

package_name = 'vision_docking'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LYH801',
    maintainer_email='yds040421@163.com',
    description='Vision-based docking system using ArUco markers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'docking_node = vision_docking.docking_node:main',
        ],
    },
)
