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
            "yolo_person_seg_node = human_yolo_seg.perception.vision.yolo_detector_node:main",
            "yolo_detector_node = human_yolo_seg.perception.vision.yolo_detector_node:main",
            "person_detector_node = human_yolo_seg.perception.vision.person_detector_node:main",
            "object_detector_node = human_yolo_seg.perception.vision.object_detector_node:main",
            "yolo_person_watch = human_yolo_seg.yolo_person_watch:main",
            "scan_person_filter_node = human_yolo_seg.perception.lidar.scan_filter_node:main",
            "person_strip_recorder_node = human_yolo_seg.processing.region.region_accumulator:main",
            "strip_saved_map_person_free = human_yolo_seg.processing.strip.strip_saved_map_person_free:main",
            "annotate_saved_map_person_overlay = human_yolo_seg.annotate_saved_map_person_overlay:main",
            "snapshot_person_regions_from_cloud = human_yolo_seg.snapshot_person_regions_from_cloud:main",
            "scan_map_colored_cloud_node = human_yolo_seg.scan_map_colored_cloud_node:main",
            "azimuth_union_node = human_yolo_seg.azimuth_union_node:main",
            "person_azimuth_markers_node = human_yolo_seg.person_azimuth_markers_node:main",
        ],
    },
)
