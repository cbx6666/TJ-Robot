from setuptools import find_packages, setup

package_name = "robot_manipulation"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    description="Mock manipulation interfaces for pick/place verification flow.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mock_pick_place_node = robot_manipulation.nodes.mock_pick_place_node:main",
        ],
    },
)
