from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'aav_gazebo'


def tree(folder):
    """Install every file under folder, keeping its layout (the models' meshes, pictures)."""
    return [(os.path.join('share', package_name, root), [os.path.join(root, f) for f in files])
            for root, _, files in os.walk(folder) if files]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        *tree('models'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AAV 2026-27 team',
    maintainer_email='aav-team@todo.invalid',
    description='Gazebo Harmonic backend behind the same topics as the real car.',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'gazebo_adapter = aav_gazebo.gazebo_adapter:main',
        ],
    },
)
