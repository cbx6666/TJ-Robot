from pathlib import Path

from setuptools import find_packages, setup

package_name = "human_yolo_seg"
_here = Path(__file__).resolve().parent
_model_dir = _here / "models"
_pt_files = sorted(_model_dir.glob("*.pt")) if _model_dir.is_dir() else []
_model_install = (
    [str(p.relative_to(_here)) for p in _pt_files]
    if _pt_files
    else ["models/.gitkeep", "models/README.txt"]
)

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/yolo_person_seg.launch.py"]),
        ("share/" + package_name, ["requirements.txt"]),
        ("share/" + package_name + "/models", _model_install),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TJ-Robot",
    description="YOLO-Seg person preview node for Gazebo/camera verification.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_person_seg_node = human_yolo_seg.yolo_person_seg_node:main",
            "yolo_person_watch = human_yolo_seg.yolo_person_watch:main",
            "scan_person_filter_node = human_yolo_seg.scan_person_filter_node:main",
        ],
    },
)
