import os
from glob import glob

from setuptools import find_packages, setup

package_name = "onboarding_python"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Joey Quinlan",
    maintainer_email="josephq02@outlook.com",
    description="odometry publisher and subscriber onboarding task",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_publisher = onboarding_python.odom_publisher:main",
            "odom_relay = onboarding_python.odom_relay:main",
        ],
    },
)
