from setuptools import setup

package_name = "robot_navigation"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    description="Navigation-specific code and configuration.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "point_to_point = robot_navigation.point_to_point:main",
            "coverage_patrol = robot_navigation.coverage_patrol:main",
            "coverage_patrol_nav2 = robot_navigation.coverage_patrol_nav2:main",
            "wall_follow_coverage = robot_navigation.wall_follow_coverage:main",
        ],
    },
)
