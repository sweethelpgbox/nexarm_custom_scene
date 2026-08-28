#!/bin/bash
# JetAuto Pro 测试程序GUI启动脚本

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 检查Python3是否安装
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到python3，请先安装Python3"
    exit 1
fi

# 检查tkinter是否安装
python3 -c "import tkinter" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "错误: 未找到tkinter模块，请安装: sudo apt-get install python3-tk"
    exit 1
fi

# 显示选择菜单
echo "========================================"
echo "JetAuto Pro 测试程序 - 可视化界面"
echo "========================================"
echo "1. 运行基础版GUI (依赖test.sh脚本)"
echo "2. 运行高级版GUI (直接执行命令)"
echo "3. 退出"
echo "========================================"
echo -n "请选择 (1-3): "
read choice

case $choice in
    1)
        echo "启动基础版GUI..."
        python3 "$SCRIPT_DIR/test_gui.py"
        ;;
    2)
        echo "启动高级版GUI..."
        python3 "$SCRIPT_DIR/test_gui_advanced.py"
        ;;
    3)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac 