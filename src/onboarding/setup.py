import os
from glob import glob

from setuptools import find_packages, setup

package_name = "onboarding"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="autonomous",
    maintainer_email="krupalhi@ualberta.ca",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_publisher = onboarding.odom_publisher:main",
            "odom_relay = onboarding.odom_relay:main",
        ],
    },
)
