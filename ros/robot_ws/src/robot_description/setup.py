from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            'share/' + package_name + '/rviz',
            ['rviz/rviz.conf.rviz']
        ),

        (
            'share/' + package_name + '/urdf',
            ['urdf/robot_rrr.urdf']
        ),

        (
            'share/' + package_name + '/meshes',
            glob('meshes/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics',
    maintainer_email='roberto.gar.1748@gmail.com',
    description='Modelo URDF y recursos visuales del robot RRR.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)