from glob import glob

from setuptools import find_packages, setup

package_name = 'delivery_bot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leon Stiffel',
    maintainer_email='leonstiffel@gmail.com',
    description='Starts the whole stack and monitors every node through heartbeats.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'health_check = delivery_bot_bringup.health_check:main',
            'health_monitor = delivery_bot_bringup.health_monitor:main',
            'stub_node = delivery_bot_bringup.stub_node:main',
            'wait_for_heartbeats = delivery_bot_bringup.wait_for_heartbeats:main',
        ],
    },
)
