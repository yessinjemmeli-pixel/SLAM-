from setuptools import setup
import os
from glob import glob

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 1. Enregistre le package dans l'index d'ament pour que 'ros2 run/launch' puissent le trouver
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),

        # 2. Installe le fichier package.xml dans le dossier partagé du package
        ('share/' + package_name, ['package.xml']),

        # 3. Installe tous les fichiers du dossier 'launch' (fichiers .py, .xml)
        # Le glob permet de détecter automatiquement tous les fichiers de lancement
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'console = robot.console:main',
            'control = robot.control:main',
            'serial = robot.serial:main',
            'lidar = robot.lidar:main',
            'automatic = robot.automatic:main',
            'localization = robot.localization:main',
            'encoders = robot.encoders:main',
        ],
    },
)
