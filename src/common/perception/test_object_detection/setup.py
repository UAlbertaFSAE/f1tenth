from glob import glob

from setuptools import find_packages, setup

package_name = "test_object_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*launch.[pxy][yma]*")),
        ("share/" + package_name + "/models", glob("models/*.[pt][trt]*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "object_detector_node = test_object_detection.object_detector_node:main"
            "image_generator_node = test_object_detection.image_generator_node:main"
        ],
    },
)
