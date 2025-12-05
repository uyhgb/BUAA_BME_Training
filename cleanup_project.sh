#!/bin/bash
# 项目重构 - 清理宇树功能包

echo "=========================================="
echo "  清理宇树冗余文件"
echo "=========================================="

# 删除宇树相关目录
echo "删除 cyclonedds_ws..."
rm -rf cyclonedds_ws

echo "删除 example (旧的)..."
rm -rf example

echo "删除 test_ws..."
rm -rf test_ws

echo "删除 docs..."
rm -rf docs

# 删除宇树相关脚本
echo "删除宇树相关脚本..."
rm -f build_and_run_custom.sh
rm -f diagnose_dds.sh
rm -f local_test.sh
rm -f setup.sh
rm -f setup_default.sh
rm -f setup_local.sh
rm -f setup_local_fixed.sh
rm -f single_terminal_test.sh
rm -f test_simulator_and_reader.sh

# 删除宇树相关文档
echo "删除宇树相关文档..."
rm -f CUSTOM_SENSOR_GUIDE.md
rm -f QUICK_FIX.md
rm -f QUICK_REFERENCE.md
rm -f TOPIC_DISCOVERY_FIX.md
rm -f "README _zh.md"
rm -f README.md

# 保留的文件
echo ""
echo "=========================================="
echo "  保留的内容:"
echo "=========================================="
echo "  ✓ 1note/              - 开发笔记"
echo "  ✓ sensor_source_code/ - ESP32传感器代码"
echo "  ✓ robot_ws/           - 新ROS2工作空间"
echo "  ✓ .devcontainer/      - Docker配置"
echo "  ✓ .github/            - CI/CD配置"
echo "  ✓ LICENSE             - 许可证"
echo ""

# 重命名新README
echo "更新README..."
mv README_NEW.md README.md

echo "=========================================="
echo "  清理完成!"
echo "=========================================="
echo ""
echo "下一步:"
echo "  1. cd robot_ws"
echo "  2. colcon build"
echo "  3. source install/setup.bash"
echo ""
