from glob import glob

from setuptools import find_packages, setup

package_name = 'delivery_bot_map_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Static maps (map.pgm + map.yaml), exported from delivery_bot_sim (Tools > City > Export Occupancy Map).
        ('share/' + package_name + '/maps/city', sorted(glob('maps/city/*.*'))),
        ('share/' + package_name + '/maps/city/tiles', sorted(glob('maps/city/tiles/*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leon Stiffel',
    maintainer_email='leonstiffel@gmail.com',
    description='Map server and map tools.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_server = delivery_bot_map_server.map_server:main',
        ],
    },
)
