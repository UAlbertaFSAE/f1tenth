from glob import glob

from setuptools import find_packages, setup

package_name = 'cone_detector_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/data', glob('data/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Krupal Shah',
    maintainer_email='krupalhi@ualberta.ca',
    description='Simulates a limited-FOV cone detector from a ground-truth cone CSV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cone_detector_sim = cone_detector_sim.generator:main',
        ],
    },
)
