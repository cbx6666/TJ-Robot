from setuptools import find_packages, setup

package_name = "robot_interaction"

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
    description="Voice and LLM interaction interfaces for task orchestration.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "voice_gateway_node = robot_interaction.nodes.voice_gateway_node:main",
            "llm_router_node = robot_interaction.nodes.llm_router_node:main",
        ],
    },
)
