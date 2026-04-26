from glob import glob

from setuptools import find_packages, setup

package_name = 'detection_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Krupal Shah',
    maintainer_email='krupalhi@ualberta.ca',
    description='Generates detections',
    license='MIT',
    entry_points={
        'console_scripts': [
            'generator = detection_generator.generator:main',
        ],
    },
)
