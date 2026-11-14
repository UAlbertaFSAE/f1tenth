from glob import glob

from setuptools import setup

setup(
    name="cone_slam",
    version="0.1.0",
    packages=["cone_slam"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/cone_slam"]),
        ("share/cone_slam", ["package.xml", "README.md", "VALIDATION.md"]),
        ("share/cone_slam/config", glob("config/*.yaml")),
        ("share/cone_slam/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    extras_require={"test": ["pytest"]},
    maintainer="UAlbertaFSAE",
    maintainer_email="fsae@ualberta.ca",
    description="Cone landmark GraphSLAM",
    license="MIT",
    entry_points={"console_scripts": ["cone_slam = cone_slam.node:main"]},
)
