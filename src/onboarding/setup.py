import os

from setuptools import find_packages, setup

package_name = "onboarding"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            ["launch/onboarding_launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="autonomous",
    maintainer_email="117860288+heibylau@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "odom_publisher = onboarding.odom_publisher:main",
            "odom_relay = onboarding.odom_relay:main",
        ],
    },
)
