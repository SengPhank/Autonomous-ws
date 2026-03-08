from setuptools import find_packages, setup

package_name = 'autonomous_costmap'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/costmap.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/costmap_params.yaml',
        ]),
        #('share/' + package_name + '/maps', [
        #    'maps/empty_map.yaml',
        #    'maps/empty_map.pgm',
        #]),
        #('share/' + package_name + '/urdf', [
        #    'urdf/rover.urdf',
        #]),
        #('share/' + package_name + '/worlds', [
        #    'worlds/simple_corridor.world',
        #]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Autonomous Team',
    author_email='user@adelaide.edu.au',
    maintainer='Autonomous Team',
    maintainer_email='user@adelaide.edu.au',
    description='Navigation Sandbox MVP for autonomous navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pcd_to_height_costmap = autonomous_costmap.pcd_to_height_costmap:main',
            'map_diagnostic = autonomous_costmap.map_diagnostic:main', 
        ],
    },
)
