import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'camera_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krupal',
    maintainer_email='krupalhi@ualberta.ca',
    description='camera detection using model',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
