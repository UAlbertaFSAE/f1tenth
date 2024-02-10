from setuptools import find_packages, setup

# "talker = lab1_pkg.talker:main",
# "relay = lab1_pkg.relay:main",

package_name = "lab1_pkg"

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
    maintainer="autonomous",
    maintainer_email="autonomous@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "talker = lab1_pkg.talker:main",
            "relay = lab1_pkg.relay:main",
        ],
    },
)
