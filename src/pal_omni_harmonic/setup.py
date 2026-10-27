from glob import glob
from setuptools import setup

package_name = 'pal_omni_harmonic'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/empty.sdf']),
        ('share/' + package_name + '/models/omni_base', [
            'models/omni_base/model.config',
            'models/omni_base/model.sdf',
            'models/omni_base/LICENSE',
        ]),
        ('share/' + package_name + '/models/omni_base/meshes/base',
         glob('models/omni_base/meshes/base/*.stl')),
        ('share/' + package_name + '/models/omni_base/meshes/laser',
         glob('models/omni_base/meshes/laser/*.stl')),
        ('share/' + package_name + '/models/omni_base/meshes/wheels',
         glob('models/omni_base/meshes/wheels/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='PAL omnidirectional robot model for Gazebo Harmonic with differential-drive test control.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'omni_controller = pal_omni_harmonic.omni_controller:main',
            'spawn_obstacle  = pal_omni_harmonic.spawn_obstacle:main',
            'safe_controller = pal_omni_harmonic.safe_controller:main',
        ],
    },
)