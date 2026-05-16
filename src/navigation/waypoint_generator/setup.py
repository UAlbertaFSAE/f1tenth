import os
from glob import glob

from setuptools import find_packages, setup

package_name = "waypoint_generator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="autonomous",
    maintainer_email="mokshdip0804@gmail.com",
    description="Waypoint triangulation node for F1TENTH",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "waypoint_generator = waypoint_generator.waypoint_generator:main"
        ],
    },
)
