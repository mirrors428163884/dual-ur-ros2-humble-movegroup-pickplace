#!/bin/bash

# ROS2 双终端一键启动脚本（带依赖检查版）
# 功能：先分析 src 下包的依赖，然后启动两个终端运行指定的 launch 文件

# --- 第一部分：依赖分析 ---
echo "=========================================="
echo "正在分析 src 目录下 ROS2 包的依赖..."
echo "=========================================="

# 检查 src 目录是否存在
if [ ! -d "src" ]; then
    echo "[警告] 当前目录下未找到 'src' 文件夹，跳过依赖分析。"
else
    # 1. 查找所有 package.xml
    # 2. 提取 <depend>, <build_depend>, <exec_depend> 标签内的内容
    # 3. 去除空白字符
    # 4. 排序并去重
    dependencies=$(find src -name "package.xml" -type f 2>/dev/null | xargs grep -h -E "<(depend|build_depend|exec_depend)>" 2>/dev/null | \
                   sed -n 's/.*<\(depend\|build_depend\|exec_depend\)> *\([^ ]*\) *<\/\1>.*/\2/p' | \
                   sort -u)

    if [ -z "$dependencies" ]; then
        echo "[提示] 未在 src 下的 package.xml 中找到标准依赖标签，或格式不匹配。"
    else
        echo "检测到以下依赖包："
        echo "$dependencies"
        echo ""
        echo "请确保上述包已通过 'rosdep install' 或 'apt' 安装。"
    fi
fi

echo "=========================================="
echo ""

# --- 第二部分：环境设置与启动 ---

# 源码当前工作目录的 ROS 设置
if [ -f "./install/setup.bash" ]; then
    source ./install/setup.bash
else
    echo "[错误] 未找到 ./install/setup.bash，请确认是否已完成编译 (colcon build)。"
    exit 1
fi

echo "启动 sim_moveit.launch.py ..."
# 在后台启动第一个 launch 文件
gnome-terminal --tab --title="Sim MoveIt Terminal" -- bash -c "
  source ./install/setup.bash;
  ros2 launch dual_ur_moveit_config sim_moveit_gazebo_all.launch.py;
  echo '';
  echo 'sim_moveit.launch.py 已结束';
  read -p '按 Enter 键关闭此窗口...'
" &

# 等待 2 秒 
echo "等待 10 秒后启动 motion_plan.launch.py ..."
sleep 10

echo "启动 motion_plan.launch.py ..."
# 打开新终端启动第二个 launch 文件
gnome-terminal --tab --title="Motion Plan Terminal" -- bash -c "
  source ./install/setup.bash;
  ros2 launch motion_plan motion_plan.launch.py;
  echo '';
  echo 'motion_plan.launch.py 已结束';
  read -p '按 Enter 键关闭此窗口...'
" &

echo "两个 launch 文件已启动！"
wait