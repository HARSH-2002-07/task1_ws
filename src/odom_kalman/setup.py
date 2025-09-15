from setuptools import find_packages, setup

package_name = 'odom_kalman'

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
    maintainer='kits',
    maintainer_email='hj075028@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kalman_node = odom_kalman.kalman_node:main',
            'test_odometry = odom_kalman.test_odometry:main',
            'wheel_controller = odom_kalman.wheel_controller:main',
        ],
    },
)
