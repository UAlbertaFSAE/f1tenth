# Copyright (c) 2026, Your Name. Subject to the MIT license.

from setuptools import find_packages, setup

package_name = "onboarding"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Benjamin Ling",
    maintainer_email="bmling@ualberta.ca",
    description="Onboarding task: drive publisher, relay node & launch",
    license="MIT",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": ["odom_publisher = onboarding.odom_publisher:main"],
    },
)
