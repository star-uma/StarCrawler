from glob import glob

from setuptools import find_packages, setup

package_name = 'starcrawler_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sim.yaml']),
        ('share/' + package_name + '/mundos', glob('mundos/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mario Garcia Jimenez',
    maintainer_email='mariogj.03@uma.es',
    description='Robot StarCrawler simulado a nivel de topicos',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = starcrawler_sim.sim_node:main',
            'mundo_node = starcrawler_sim.mundo_node:main',
            'mundo_check = starcrawler_sim.mundo_check:main',
        ],
    },
)
