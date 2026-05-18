from setuptools import setup
import os
from glob import glob

package_name = 'rh8d_mujoco_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labauto',
    maintainer_email='ehtishamashraf67@gmail.com',
    description='RH8D MuJoCo control package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'rh8dR_read_sensors = rh8d_mujoco_control.rh8dR_read_sensors:main',
        'rh8dR_simple_control = rh8d_mujoco_control.rh8dR_simple_control:main',
        'rh8dR_object_pick = rh8d_mujoco_control.rh8dR_object_pick:main',
        'rh8dR_object_pick_JointTrajectory = rh8d_mujoco_control.rh8dR_object_pick_JointTrajectory:main',
    ],
    },
)