# Go2 黑色线迹巡线器 (Line Tracker)

基于 [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)、OpenCV 和 RealSense D435i 的 Unitree Go2 机器狗地面黑色线迹巡线程序。

- 🖤 **线迹跟踪** — 全帧行扫描 + 最小二乘直线拟合，提取线的位置偏移和方向角度
- 🎯 **智能弯道** — 角度 >50° 判定为弯道，切换方向搜索模式
- 🔄 **丢失搜索** — 顺着上次方向旋转搜索 + 慢速前进
- 📷 **D435i TCP 中继** — D435i 接在 Jetson Orin Nano 上，TCP 转发图像到笔记本

---

## 控制逻辑

```
直线/缓弯 (angle ≤ 50°):
  vyaw = -(kp_pos × offset − kp_angle × angle)   双 P 控制
  vx   = base_vx - 弯道减速                         弯道自动降速

直角弯 (angle > 50°):
  vyaw = ±search_vyaw   顺着 last_line_angle 方向固定旋转
  vx   = base_vx × 0.6  慢速前进

丢失 (0 个扫描点):
  vyaw = ±search_vyaw   朝 last_line_offset 方向旋转搜索
  vx   = 0               停止
```

---

## 系统架构

```
Jetson Orin Nano (192.168.123.18)      开发笔记本
┌────────────────────────┐             ┌─────────────────────────┐
│ D435i → d435i_relay    │  TCP :9191  │ black_line_tracker      │
│ (librealsense2)        │────────────→│  recv() → cv::Mat       │
│                         │  BGR 图像   │  → DetectLine()         │
│ Go2 运动控制 ←─────────┼──SDK2 DDS─│  → SportClient → Move() │
└────────────────────────┘             └─────────────────────────┘
```

主循环每帧：**采集 → 全帧灰度二值化 → 行扫描 → 直线拟合 → 弯道检测 → 控制 → 显示**。

---

## 依赖

| 依赖 | 用途 | 安装说明 |
|------|------|----------|
| Unitree SDK2 | DDS 通信 + 运动控制 | [GitHub](https://github.com/unitreerobotics/unitree_sdk2) |
| OpenCV (≥4.x) | 图像处理 + GUI | `sudo apt install libopencv-dev` |
| CMake (≥3.10) | 构建系统 | `sudo apt install cmake` |
| C++17 编译器 | 编译 | `sudo apt install build-essential` |

Jetson 额外依赖：`sudo apt install ros-humble-librealsense2`

---

## 编译

### 默认模式（Go2 内置摄像头）

```bash
mkdir build && cd build
cmake .. -DUNITREE_SDK2_ROOT=/path/to/unitree_sdk2
make -j$(nproc)
```

### D435i 模式（TCP 中继）

```bash
mkdir build && cd build
cmake .. -DUSE_REALSENSE=ON -DUNITREE_SDK2_ROOT=/path/to/unitree_sdk2
make -j$(nproc)
```

编译产物：

| 目标 | 说明 |
|------|------|
| `build/black_line_tracker` | 巡线主程序（笔记本运行） |
| `build/d435i_relay` | TCP 中继（x86_64，传到 Jetson 需在 ARM64 上重新编译） |

---

## 部署与运行

### 1. Jetson 上编译并运行中继

```bash
# 将 d435i_relay.cpp 传到 Jetson
scp build/d435i_relay.cpp unitree@192.168.123.18:~/

# Jetson 上本地编译（ARM64）
ssh unitree@192.168.123.18
g++ -std=c++17 d435i_relay.cpp -lrealsense2 -lpthread -o d435i_relay

# 运行（D435i 已 USB 连接）
./d435i_relay
```

### 2. 笔记本上运行

```bash
# 基本用法
./run_black_line_tracker.sh enp8s0

# 仅检测、调参
./run_black_line_tracker.sh enp8s0 --no-move
```

---

## 界面说明

弹出 2 个 OpenCV 窗口：

| 窗口名 | 内容 |
|--------|------|
| **Line Tracker** | 主画面，叠加拟合线、扫描点、控制状态 |
| **Threshold (调参)** | 二值化掩码 + Trackbar 滑块 |

### 主画面标注

| 标记 | 含义 |
|------|------|
| 蓝色竖线 | 画面垂直中线（偏移量零点） |
| 绿色小圆点 | 各行扫描到的黑色段中点 |
| 绿色实线 | 最小二乘拟合线迹 |
| 左上角 | `LINE off:... ang:...° vx:... vyaw:...` |
| 橙色文字 | `LINE LOST ... SEARCHING` |

---

## 检测原理

1. 全帧灰度化 → 固定阈值二值化（THRESH_BINARY_INV）→ 形态学去噪
2. 等间距扫描 6 行（从下到上），每行找最长黑色段的中点
3. 最小二乘直线拟合 $x = a \cdot y + b$
4. 提取 $offset = x_{bottom} - center\_x$ 和 $angle = \arctan(a)$
5. 角度 >50° 进入弯道模式，用进弯前保存的方向旋转搜索

---

## 调参

| Trackbar | 范围 | 默认 | 说明 |
|----------|------|------|------|
| **Threshold** | 0~255 | 80 | 二值化阈值 |
| **Morph Size** | 1~21 | 5 | 形态学核大小 |
| **ROI %** | 5~100 | 100 | 全帧检测 |
| **Scan Rows** | 2~20 | 6 | 扫描行数 |

控制参数（`TrackParams` 结构体）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_pos` | 0.01 | 位置 P 系数 |
| `kp_angle` | 0.8 | 角度 P 系数 |
| `max_vyaw` | 2.5 | 最大转向角速度 (rad/s) |
| `base_vx` | 0.35 | 基准前进速度 (m/s) |
| `curve_vx_min` | 0.12 | 弯道最低速度 (m/s) |
| `search_vyaw` | 0.8 | 搜索旋转速度 (rad/s) |
| `kCornerAngle` | 0.87 (50°) | 弯道判定阈值 |

---

## 故障排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| `GetImageSample 失败` | 内置摄像头模式，网络不通 | 检查网线 / 接口名 |
| 连不上 TCP 中继 | Jetson 上未运行 relay | SSH 检查 `./d435i_relay` |
| `Exec format error` | ARM64 / x86_64 不匹配 | 在 Jetson 上本地编译 |
| 检测不到线 | 阈值不匹配 | `--no-move` 调 Threshold |
| 弯道过不去 | `kCornerAngle` 或速度不合适 | 调整参数 |

---

## 文件结构

```
black_line_tracker/
├── CMakeLists.txt              # 构建配置
├── README.md
├── run_black_line_tracker.sh   # 启动脚本
└── src/
    ├── black_line_tracker.cpp  # 巡线主程序
    └── d435i_relay.cpp         # D435i TCP 中继
```

---

## 许可

MIT License
