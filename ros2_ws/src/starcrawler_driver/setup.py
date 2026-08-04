from setuptools import find_packages, setup

package_name = 'starcrawler_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/driver.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mario Garcia Jimenez',
    maintainer_email='mariogj.03@uma.es',
    description='Puente ROS 2 <-> ESP32 del robot StarCrawler',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_node = starcrawler_driver.driver_node:main',
        ],
    },
)
