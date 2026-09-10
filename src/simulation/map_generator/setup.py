from glob import glob

from setuptools import find_packages, setup

package_name = "map_generator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Krupal Shah",
    maintainer_email="krupalhi@ualberta.ca",
    description="Standalone track editor GUI and ground-truth track marker publisher",
    license="MIT",
    entry_points={
        "console_scripts": [
            "map_generator_gui = map_generator.app:main",
            "track_map_publisher = map_generator.track_map_publisher:main",
        ],
    },
)
