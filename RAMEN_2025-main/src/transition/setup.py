from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'transition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # find_packages()は使わない
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='araishogo',
    maintainer_email='araishogo@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transition_tutorial = transition.transition_tutorial:main',
            'wall_follower_lifecycle = transition.wall_follwer_lifecycle:main',
        ],
    },
)
