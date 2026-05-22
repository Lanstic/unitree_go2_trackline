#!/bin/bash
#
# Go2 巡线环境安装脚本
# ================================
# 安装 Unitree SDK2 Python 和所有依赖
#

set -e

echo "=========================================="
echo "  Go2 巡线环境安装脚本"
echo "=========================================="

# 检测 Python 版本
PYTHON_CMD=""
if command -v python3 &> /dev/null; then
    PYTHON_CMD="python3"
elif command -v python &> /dev/null; then
    PYTHON_CMD="python"
else
    echo "错误: 未找到 Python"
    exit 1
fi

PYTHON_VERSION=$($PYTHON_CMD --version)
echo "使用 Python: $PYTHON_VERSION"

# 检查是否为虚拟环境
if [ -z "$VIRTUAL_ENV" ]; then
    echo ""
    echo "建议在虚拟环境中安装，运行以下命令:"
    echo "  python3 -m venv venv"
    echo "  source venv/bin/activate"
    echo "  bash setup_env.sh"
    echo ""
    read -p "是否继续在系统环境中安装? (y/n): " confirm
    if [ "$confirm" != "y" ]; then
        exit 0
    fi
fi

echo ""
echo "[1/5] 更新 pip..."
$PYTHON_CMD -m pip install --upgrade pip

echo ""
echo "[2/5] 安装 OpenCV 和 NumPy..."
$PYTHON_CMD -m pip install opencv-python opencv-python-headless numpy

echo ""
echo "[3/5] 安装 Intel RealSense SDK (pyrealsense2)..."
$PYTHON_CMD -m pip install pyrealsense2

echo ""
echo "[4/5] 下载并安装 Unitree SDK2 Python..."
SDK2_DIR="$PWD/unitree_sdk2_python"

if [ -d "$SDK2_DIR" ]; then
    echo "SDK2 目录已存在，更新中..."
    cd "$SDK2_DIR"
    git pull 2>/dev/null || echo "无法更新，请手动删除后重新安装"
    cd - > /dev/null
else
    echo "克隆 Unitree SDK2 Python..."
    git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
    cd "$SDK2_DIR"
    git submodule update --init --recursive
    cd - > /dev/null
fi

echo ""
echo "[5/5] 安装 SDK2 到当前 Python 环境..."
cd "$SDK2_DIR"
$PYTHON_CMD -m pip install -e .
cd - > /dev/null

echo ""
echo "=========================================="
echo "  安装完成!"
echo "=========================================="
echo ""
echo "验证安装:"
echo "  $PYTHON_CMD -c \"import cv2; import numpy; import pyrealsense2 as rs; print('OpenCV:', cv2.__version__); print('NumPy:', numpy.__version__); print('pyrealsense2: OK')\""
echo ""
echo "验证 SDK2:"
echo "  $PYTHON_CMD -c \"from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('SDK2: OK')\""
echo ""
echo "运行巡线程序:"
echo "  python3 trackline.py"
