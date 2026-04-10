本目录用于存放 coverage_patrol 在「路径走完后」自动保存的地图（.pgm + .yaml）。

默认行为（见 robot_navigation README）：
- 在仓库根目录下启动 ros2 且未指定 save_map_directory 时，保存到本目录。
- 或通过环境变量：export TJ_ROBOT_SAVED_MAPS_DIR=/绝对路径/saved_maps
- 或参数：-p save_map_directory:=/你的路径

仓库已忽略 *.pgm / *.yaml，避免误提交大文件；需要版本管理时请自行 force add。
