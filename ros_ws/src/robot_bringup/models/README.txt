================================================================================
动态障碍物「人形」模型 person_standing — 原因说明与安装教程
================================================================================

【为什么需要单独准备】

moving_obstacle.sdf 使用 Gazebo 模型 URI：
  model://person_standing/meshes/standing.dae

person_standing 来自 OSRF 的 gazebo_models 仓库，不是 ROS / TurtleBot3
安装包的一部分。仅 git clone 本仓库后执行 tb3_stack.sh，若未准备该模型，
可能出现：人形不显示、碰撞异常、或 gzserver / spawn 日志中报找不到 mesh。

这与 TB3_STACK_MODE=assist、是否 colcon build 无直接关系；缺的是 Gazebo
可解析的模型目录（含 model.config 与 meshes/standing.dae）。

tb3_stack.sh 会将以下路径（若存在）prepend 到 GAZEBO_MODEL_PATH：
  - ros_ws/src/robot_bringup/models
  - install/share/robot_bringup/models（colcon 安装后）
  - ~/.gazebo/models

--------------------------------------------------------------------------------
做法一：放到用户目录 ~/.gazebo/models（推荐）
--------------------------------------------------------------------------------

在「运行 bash scripts/tb3_stack.sh」的同一环境执行（WSL 里跑仿真就用 WSL
终端，不要用只放在 Windows E:\ 下却从未同步到 WSL 的目录）。

  mkdir -p ~/.gazebo/models
  cd ~/.gazebo/models
  rm -rf _gm
  git clone --depth 1 https://github.com/osrf/gazebo_models.git _gm
  mv _gm/person_standing .
  rm -rf _gm

然后重新：bash scripts/tb3_stack.sh start

若提示 destination path '_gm' already exists：先执行 rm -rf ~/.gazebo/models/_gm
再重新 clone（上次失败可能留下空目录或不完整仓库）。

若 clone 末尾报错（如 could not open .../objects/pack/tmp_pack_...、
invalid index-pack）：
  - 检查磁盘：df -h /home /tmp
  - 改在 /tmp 克隆再搬回家目录：
      cd /tmp
      rm -rf gazebo_models
      git clone --depth 1 https://github.com/osrf/gazebo_models.git gazebo_models
      mv gazebo_models/person_standing ~/.gazebo/models/
      rm -rf gazebo_models
  - 或浏览器下载 zip（仅拷 person_standing 文件夹）：
      https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip
    解压后将 person_standing 整个目录放到 ~/.gazebo/models/person_standing/
    （应存在 ~/.gazebo/models/person_standing/model.config）

仅需要 person_standing 时可尝试稀疏检出（减小下载量，在 /tmp 执行）：
  cd /tmp
  rm -rf gm
  git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/osrf/gazebo_models.git gm
  cd gm
  git sparse-checkout set person_standing
  git checkout 2>/dev/null || git read-tree -mu HEAD
  cp -r person_standing ~/.gazebo/models/
  cd .. && rm -rf gm

--------------------------------------------------------------------------------
做法二：放进本包 models 目录（与仓库一起维护）
--------------------------------------------------------------------------------

将 gazebo_models 仓库中的整个 person_standing 文件夹复制到：

  ros_ws/src/robot_bringup/models/person_standing/

（内含 model.config 与 meshes/ 等。）

然后：

  cd ros_ws
  source /opt/ros/humble/setup.bash
  colcon build --packages-select robot_bringup
  source install/setup.bash

--------------------------------------------------------------------------------
验证
--------------------------------------------------------------------------------

  test -f ~/.gazebo/models/person_standing/model.config && echo OK
  test -f ~/.gazebo/models/person_standing/meshes/standing.dae && echo OK

（若使用做法二，把路径换成 ros_ws/src/robot_bringup/models/person_standing/）

仍异常时查看：/tmp/tb3_stack/gzserver.log、spawn_obstacle.log 是否仍有
person_standing 或 mesh 相关错误。
