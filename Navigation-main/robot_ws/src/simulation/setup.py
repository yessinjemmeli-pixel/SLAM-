from setuptools import find_packages, setup

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pygame', 'pymunk'],
    zip_safe=True,
    maintainer='f',
    maintainer_email='f@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest',],
    },
    entry_points={
        'console_scripts': [
            'simulation = simulation.simulation:main',
            ],
    },
)
