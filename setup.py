from setuptools import find_packages, setup


setup(
    packages=find_packages(
        include=[
            "robot_core*",
            "robot_perception*",
            "robot_mapping*",
            "robot_navigation*",
            "robot_interaction*",
            "robot_tasks*",
            "robot_experiments*",
        ]
    ),
)
