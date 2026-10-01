from setuptools import find_packages, setup

package_name = 'starcrawler_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/odometry.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mario Garcia Jimenez',
    maintainer_email='mariogj.03@uma.es',
    description='Odometria de orugas y apoyo del chasis de StarCrawler',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = starcrawler_odometry.odometry_node:main',
            'chasis_node = starcrawler_odometry.chasis_node:main',
            'bandas_node = starcrawler_odometry.bandas_node:main',
        ],
    },
)
