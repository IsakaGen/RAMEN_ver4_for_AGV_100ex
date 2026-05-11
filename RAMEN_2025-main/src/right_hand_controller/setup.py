from setuptools import setup

package_name = 'right_hand_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # find_packages()は使わない
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shota Simazaki',
    maintainer_email='7521059@ed.tus.ac.jp',
    description='AGV を右手法で自己運転するコントローラ',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower2 = right_hand_controller.wall_follower2:main'
        ],
    },
)
