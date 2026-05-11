from setuptools import setup

package_name = 'encoder_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # ← モジュールでなく、パッケージを指定！
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Encoder odometry node for differential drive robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_odometry_node = encoder_odometry.encoder_odometry_node:main',
        ],
    },
)
