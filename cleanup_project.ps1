# PowerShell清理脚本
# 项目重构 - 清理宇树功能包

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "  清理宇树冗余文件" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# 删除宇树相关目录
Write-Host "删除 cyclonedds_ws..." -ForegroundColor Yellow
Remove-Item -Path "cyclonedds_ws" -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "删除 example (旧的)..." -ForegroundColor Yellow
Remove-Item -Path "example" -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "删除 test_ws..." -ForegroundColor Yellow
Remove-Item -Path "test_ws" -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "删除 docs..." -ForegroundColor Yellow
Remove-Item -Path "docs" -Recurse -Force -ErrorAction SilentlyContinue

# 删除宇树相关脚本
Write-Host "删除宇树相关脚本..." -ForegroundColor Yellow
$scripts = @(
    "build_and_run_custom.sh",
    "diagnose_dds.sh",
    "local_test.sh",
    "setup.sh",
    "setup_default.sh",
    "setup_local.sh",
    "setup_local_fixed.sh",
    "single_terminal_test.sh",
    "test_simulator_and_reader.sh"
)

foreach ($script in $scripts) {
    Remove-Item -Path $script -Force -ErrorAction SilentlyContinue
}

# 删除宇树相关文档
Write-Host "删除宇树相关文档..." -ForegroundColor Yellow
$docs = @(
    "CUSTOM_SENSOR_GUIDE.md",
    "QUICK_FIX.md",
    "QUICK_REFERENCE.md",
    "TOPIC_DISCOVERY_FIX.md",
    "README _zh.md",
    "README.md"
)

foreach ($doc in $docs) {
    Remove-Item -Path $doc -Force -ErrorAction SilentlyContinue
}

# 保留的文件
Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "  保留的内容:" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "  ✓ 1note/              - 开发笔记" -ForegroundColor Green
Write-Host "  ✓ sensor_source_code/ - ESP32传感器代码" -ForegroundColor Green
Write-Host "  ✓ robot_ws/           - 新ROS2工作空间" -ForegroundColor Green
Write-Host "  ✓ .devcontainer/      - Docker配置" -ForegroundColor Green
Write-Host "  ✓ .github/            - CI/CD配置" -ForegroundColor Green
Write-Host "  ✓ LICENSE             - 许可证" -ForegroundColor Green
Write-Host ""

# 重命名新README
Write-Host "更新README..." -ForegroundColor Yellow
if (Test-Path "README_NEW.md") {
    Move-Item -Path "README_NEW.md" -Destination "README.md" -Force
}

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "  清理完成!" -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "下一步:" -ForegroundColor Yellow
Write-Host "  1. cd robot_ws" -ForegroundColor White
Write-Host "  2. colcon build" -ForegroundColor White
Write-Host "  3. source install/setup.bash" -ForegroundColor White
Write-Host ""

Read-Host "按Enter键退出"
