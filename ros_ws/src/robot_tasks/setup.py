from setuptools import find_packages, setup

package_name = "robot_tasks"

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
    description="Task orchestration and mission logic.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "task_manager_node = robot_tasks.nodes.task_manager_node:main",
        ],
    },
)
