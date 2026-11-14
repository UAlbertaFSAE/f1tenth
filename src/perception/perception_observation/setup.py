from setuptools import setup

setup(
    name="perception_observation",
    version="0.1.0",
    packages=["perception_observation"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/perception_observation"],
        ),
        ("share/perception_observation", ["package.xml", "README.md", "VALIDATION.md"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    maintainer="UAlbertaFSAE",
    maintainer_email="fsae@ualberta.ca",
    description="Capture-time cone observation transport",
    license="MIT",
)
