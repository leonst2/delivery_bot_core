from setuptools import find_packages, setup

package_name = 'delivery_bot_viz'

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
    description='Debug visualization.',
    license='Proprietary',
    tests_require=['pytest'],
)
