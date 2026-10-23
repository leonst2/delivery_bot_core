from setuptools import find_packages, setup

package_name = 'delivery_bot_grid_step'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leon Stiffel',
    maintainer_email='leonstiffel@gmail.com',
    description='Deterministic grid-step controller for the delivery_bot.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_step_controller = delivery_bot_grid_step.grid_step_controller:main',
        ],
    },
)
