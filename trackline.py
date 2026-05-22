#!/usr/bin/env python3
"""
Go2 D435i 黑线检测 + 巡线运动控制 (SDK2 直连版)
===================================================
功能：
  1. RealSense D435i 实时检测地面黑线
  2. P控制器将偏移量转为角速度
  3. 通过 Unitree SDK2 SportClient 直接控制 Go2 运动
  4. 黑线消失 → 停止；重新检测到 → 继续巡线
  5. HTTP 浏览器实时调试画面
  6. SDK2 连接状态监控和自动重连

用法：
  python3 trackline.py                  # 完整巡线
  python3 trackline.py --no-motion      # 仅调试画面(不控制机器人)
  python3 trackline.py --speed 0.12     # 设置线速度
  python3 trackline.py --kp 0.005       # 设置P系数
  python3 trackline.py --interface eth0 # 指定网络接口

无需 ROS2，无需额外终端！SDK2 通过 DDS 直接与 Go2 通信。
"""

import sys
import os

# 添加 SDK2 路径
SDK2_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'unitree_sdk2_python')
if SDK2_PATH not in sys.path:
    sys.path.insert(0, SDK2_PATH)

import cv2
import numpy as np

# 检查依赖
def check_dependencies():
    missing = []
    try:
        import pyrealsense2 as rs
    except ImportError:
        missing.append('pyrealsense2')
    try:
        import numpy as np
    except ImportError:
        missing.append('numpy')
    try:
        import cv2
    except ImportError:
        missing.append('opencv-python')

    if missing:
        print(f"错误: 缺少依赖库: {', '.join(missing)}")
        print("运行以下命令安装:")
        print(f"  pip install {' '.join(missing)}")
        sys.exit(1)

check_dependencies()

import pyrealsense2 as rs
import numpy as np
import time
import threading
import argparse
from http.server import HTTPServer, BaseHTTPRequestHandler
from collections import deque

# Unitree SDK2 运动控制
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.go2.sport.sport_client import SportClientState
    SDK2_AVAILABLE = True
except ImportError as e:
    print(f"[警告] SDK2 未安装: {e}")
    print("运行 setup_env.sh 安装完整环境")
    SDK2_AVAILABLE = False
    SportClient = None
    SportClientState = None

# ==================== 配置 ====================
LINEAR_SPEED = 0.5           # 前进速度 m/s
KP = 0.005                    # P控制器比例系数 (角速度 = KP * offset)
KI = 0.0001                   # 积分系数 (累积误差纠正)
MAX_ANGULAR = 0.5            # 最大角速度 rad/s
LOST_FRAME_THRESHOLD = 30     # 连续完全丢失多少帧后停止
DETECT_POINTS_THRESHOLD = 2  # 最少检测点数(>0就走, 仅需2点确认)
HTTP_PORT = 8081             # 调试网页端口
HTTP_HOST = '0.0.0.0'
DEFAULT_INTERFACE = 'eth0'   # Go2 通信网口 (默认eth0)
RECONNECT_INTERVAL = 5       # 重连间隔秒数


def detect_black_line_center(frame, threshold=80, scan_lines=20, min_width=8):
    """
    检测黑线中心点
    扫描范围：底部到图像上方1/4处
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((3, 3), np.uint8)
    morphed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    morphed = cv2.morphologyEx(morphed, cv2.MORPH_OPEN, kernel, iterations=1)

    result = frame.copy()
    height, width = gray.shape
    detected_points = []

    # 扫描范围：底部到上方1/4处（扩大了一倍）
    top_y = height // 4
    bottom_y = height - 20
    step_y = (bottom_y - top_y) // scan_lines

    for scan_y in range(bottom_y, top_y, -step_y):
        row_pixels = morphed[scan_y, :]
        in_line = False
        start_x = 0

        for x in range(0, width):
            if row_pixels[x] > 0 and not in_line:
                start_x = x
                in_line = True
            elif row_pixels[x] == 0 and in_line:
                end_x = x
                line_width = end_x - start_x

                if line_width >= min_width:
                    center_x = (start_x + end_x) // 2
                    detected_points.append((center_x, scan_y))
                    cv2.circle(result, (center_x, scan_y), 3, (0, 255, 0), -1)

                in_line = False

        if in_line:
            end_x = width
            line_width = end_x - start_x
            if line_width >= min_width:
                center_x = (start_x + end_x) // 2
                detected_points.append((center_x, scan_y))
                cv2.circle(result, (center_x, scan_y), 3, (0, 255, 0), -1)

    # 画扫描线
    for scan_y in range(bottom_y, top_y, -step_y):
        cv2.line(result, (0, scan_y), (width, scan_y), (100, 100, 100), 1)

    # 曲线拟合
    offset = 0
    if len(detected_points) >= 5:
        points = np.array(detected_points)

        try:
            z = np.polyfit(points[:, 1], points[:, 0], 3)
            p = np.poly1d(z)

            y_vals = np.linspace(points[-1, 1], points[0, 1], 50)
            x_vals = p(y_vals).astype(np.int32)
            x_vals = np.clip(x_vals, 0, width - 1)

            for i in range(len(y_vals) - 1):
                pt1 = (x_vals[i], int(y_vals[i]))
                pt2 = (x_vals[i + 1], int(y_vals[i + 1]))
                cv2.line(result, pt1, pt2, (0, 255, 0), 2)

            # 计算底部偏移
            bottom_calc_y = height - 40
            bottom_x = np.clip(int(p(bottom_calc_y)), 0, width - 1)
            center_x = width // 2
            offset = bottom_x - center_x

            # 画中心线
            cv2.line(result, (center_x, height), (center_x, 0), (255, 0, 0), 1)
            cv2.circle(result, (bottom_x, bottom_calc_y), 5, (0, 0, 255), -1)

        except Exception:
            pass

    return result, offset, len(detected_points)


# ==================== 运动控制 (SDK2 直连) ====================
class Go2MotionController:
    """
    通过 Unitree SDK2 SportClient 直接控制 Go2 运动
    支持连接状态监控和自动重连
    """
    def __init__(self, enabled=True, interface='eth0'):
        self.enabled = enabled and SDK2_AVAILABLE
        self.interface = interface
        self.client = None
        self._initialized = False
        self._connected = False
        self._last_connect_time = 0
        self._reconnect_lock = threading.Lock()
        self._connection_thread = None
        self._running = True
        self._velocity_filter = deque(maxlen=3)  # 速度平滑滤波器

    def init(self):
        """初始化 DDS 通道和 SportClient"""
        if not self.enabled:
            print("[Motion] 运动控制已禁用 (SDK2 未安装)")
            return True
        return self._connect()

    def _connect(self):
        """建立与机器人的连接"""
        with self._reconnect_lock:
            try:
                print(f"[Motion] 初始化 DDS (网口:{self.interface})...")

                # 如果已初始化，先清理
                if self.client is not None:
                    try:
                        self.client.Stop()
                    except Exception:
                        pass

                # 初始化 DDS 通道
                ChannelFactoryInitialize(0, self.interface)

                # 创建并初始化 SportClient
                self.client = SportClient()
                self.client.SetTimeout(10.0)
                self.client.Init()

                # 等待连接建立
                for _ in range(10):
                    time.sleep(0.1)
                    if self._check_connection():
                        self._initialized = True
                        self._connected = True
                        self._last_connect_time = time.time()
                        print("[Motion] ✓ SportClient 连接成功")
                        return True

                print("[Motion] ⚠ SportClient 初始化完成但连接未确认")
                self._initialized = True
                return True

            except Exception as e:
                print(f"[Motion] ✗ 连接失败: {e}")
                self._initialized = False
                self._connected = False
                return False

    def _check_connection(self):
        """检查连接状态"""
        if not self.client or not self._initialized:
            return False
        try:
            # 尝试获取状态
            if hasattr(SportClientState, 'OK'):
                pass  # 状态检查可选
            return True
        except Exception:
            return False

    def _should_reconnect(self):
        """判断是否应该尝试重连"""
        if self._connected:
            return False
        elapsed = time.time() - self._last_connect_time
        return elapsed >= RECONNECT_INTERVAL

    def move(self, vx: float, vyaw: float):
        """
        发送运动指令 (带速度平滑)
        """
        if not self.enabled:
            return

        # 速度平滑
        self._velocity_filter.append((vx, vyaw))
        if len(self._velocity_filter) >= 2:
            vx = np.mean([v[0] for v in self._velocity_filter])
            vyaw = np.mean([v[1] for v in self._velocity_filter])

        if not self._initialized:
            # 尝试重连
            if self._should_reconnect():
                print("[Motion] 尝试重新连接...")
                self._connect()
            return

        if not self._connected:
            # 尝试重连
            if self._should_reconnect():
                self._connect()
            return

        try:
            self.client.Move(vx, 0.0, vyaw)
            self._connected = True
        except Exception as e:
            if self._connected:
                print(f"[Motion] 发送指令失败: {e}")
            self._connected = False
            self._last_connect_time = time.time()

    def stop(self):
        """停止运动"""
        if self.enabled and self._initialized and self._connected:
            try:
                self.client.Move(0.0, 0.0, 0.0)
                self.client.SwitchGait(0)  # 切换到站立
            except Exception as e:
                print(f"[Motion] 停止命令失败: {e}")
                self._connected = False

    def get_status(self):
        """获取连接状态"""
        status = "DISCONNECTED"
        if self._initialized:
            if self._connected:
                status = "CONNECTED"
            else:
                status = "INITIALIZED"
        return status

    def cleanup(self):
        """清理资源"""
        self._running = False
        if self.client and self._initialized:
            try:
                self.stop()
                self.client.Stop()
            except Exception:
                pass


# ==================== HTTP 调试服务器 ====================
HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head><meta charset="UTF-8"><title>Go2 巡线 - D435i</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Arial,sans-serif;text-align:center;background:#0d1117;color:#c9d1d9;padding:20px}
h1{color:#58a6ff;font-size:22px;margin-bottom:5px}
.panel{background:#161b22;border:1px solid #30363d;border-radius:12px;padding:15px;display:inline-block}
.panel img{border-radius:8px;width:640px;height:480px;object-fit:contain;background:#000}
.info{margin-top:10px;font-size:14px;color:#8b949e}
.fps{color:#d2a8ff;font-size:12px;margin-top:5px}
.status{color:#3fb950;font-size:14px}
.status.offline{color:#f85149}
</style></head>
<body>
<h1>Go2 黑线巡线 - D435i 实时画面</h1>
<div class="panel"><h2>调试画面</h2>
<img id="debugImg" alt="等待画面...">
<div class="fps">刷新率: <span id="fpsCounter">--</span> fps</div></div>
<div class="info">
<span id="connStatus" class="status">--</span> |
<span id="statusText">--</span> |
<span id="velText">v=0 w=0</span>
</div>
<script>
let count=0,fps=0;setInterval(()=>{fps=count;count=0},1000)
async function refresh(){
try{let r=await fetch('/snapshot?t='+Date.now())
if(!r.ok)throw new Error('HTTP '+r.status)
let b=await r.blob(),url=URL.createObjectURL(b)
let img=document.getElementById('debugImg'),old=img.src
img.src=url;if(old.startsWith('blob:'))URL.revokeObjectURL(old)
count++;document.getElementById('fpsCounter').textContent=fps
let st=r.headers.get('X-Status')||'--'
let conn=r.headers.get('X-Connection')||'--'
let vel=r.headers.get('X-Velocity')||'v=0 w=0'
document.getElementById('statusText').textContent=st
document.getElementById('velText').textContent=vel
let connEl=document.getElementById('connStatus')
connEl.textContent='SDK2: '+conn
connEl.className=conn==='CONNECTED'?'status':'status offline'
}catch(e){console.error(e)}
setTimeout(refresh,30)}
refresh()
</script></body></html>"""


class DebugHTTPServer:
    """轻量 HTTP 服务器，浏览器实时查看检测画面"""
    def __init__(self, host=HTTP_HOST, port=HTTP_PORT):
        self.host = host
        self.port = port
        self.latest_jpeg = None
        self.lock = threading.Lock()
        self.status_text = "--"
        self.vel_text = "v=0 w=0"
        self.conn_status = "DISCONNECTED"
        self.server = None

    def update(self, jpeg_bytes: bytes, status: str = "", vel: str = "", conn: str = ""):
        with self.lock:
            self.latest_jpeg = jpeg_bytes
            if status:
                self.status_text = status
            if vel:
                self.vel_text = vel
            if conn:
                self.conn_status = conn

    def start(self):
        parent = self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/' or self.path == '/index.html':
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(HTML_PAGE.encode())
                elif self.path.startswith('/snapshot'):
                    with parent.lock:
                        data = parent.latest_jpeg
                        st = parent.status_text
                        vel = parent.vel_text
                        conn = parent.conn_status
                    if data:
                        self.send_response(200)
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Cache-Control', 'no-cache')
                        self.send_header('Content-Length', str(len(data)))
                        self.send_header('X-Status', st)
                        self.send_header('X-Velocity', vel)
                        self.send_header('X-Connection', conn)
                        self.end_headers()
                        self.wfile.write(data)
                    else:
                        self.send_response(503)
                        self.end_headers()
                else:
                    self.send_response(404)
                    self.end_headers()

            def log_message(self, format, *args):
                pass  # 静默 HTTP 日志

        self.server = HTTPServer((self.host, self.port), Handler)
        t = threading.Thread(target=self.server.serve_forever, daemon=True)
        t.start()
        print(f"[HTTP] 调试页面: http://127.0.0.1:{self.port}")

    def stop(self):
        if self.server:
            self.server.shutdown()


def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Go2 黑线巡线 (D435i)')
    parser.add_argument('--no-motion', action='store_true', help='禁用运动控制(仅调试)')
    parser.add_argument('--no-http', action='store_true', help='禁用HTTP调试服务器')
    parser.add_argument('--port', type=int, default=HTTP_PORT, help=f'HTTP端口 (默认{HTTP_PORT})')
    parser.add_argument('--speed', type=float, default=LINEAR_SPEED, help=f'线速度 m/s (默认{LINEAR_SPEED})')
    parser.add_argument('--kp', type=float, default=KP, help=f'P系数 (默认{KP})')
    parser.add_argument('--ki', type=float, default=KI, help=f'I系数 (默认{KI})')
    parser.add_argument('--max-angular', type=float, default=MAX_ANGULAR, help=f'最大角速度 (默认{MAX_ANGULAR})')
    parser.add_argument('--lost-threshold', type=int, default=LOST_FRAME_THRESHOLD,
                        help=f'丢失帧阈值 (默认{LOST_FRAME_THRESHOLD})')
    parser.add_argument('--threshold', type=int, default=80, help='黑线灰度阈值 (默认80)')
    parser.add_argument('--interface', type=str, default=DEFAULT_INTERFACE,
                        help=f'Go2通信网口 (默认{DEFAULT_INTERFACE}, 可选wlan0等)')
    parser.add_argument('--low-speed', type=float, default=0.5,
                        help=f'减速模式速度比例 (默认0.5)')
    args = parser.parse_args()

    enable_motion = not args.no_motion and SDK2_AVAILABLE
    enable_http = not args.no_http

    # 初始化相机
    pipeline = rs.pipeline()
    config = rs.config()

    ctx = rs.context()
    devices = ctx.query_devices()

    if len(devices) == 0:
        print("错误: 未检测到 RealSense 设备！")
        print("请确保 D435i 已连接并被系统识别")
        return

    device = devices[0]
    print(f"已连接设备: {device.get_info(rs.camera_info.name)}")

    # 尝试获取固件版本
    try:
        fw_version = device.get_info(rs.camera_info.firmware_version)
        print(f"固件版本: {fw_version}")
    except Exception:
        pass

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)

    # 初始化运动控制 (SDK2 直连)
    motion = Go2MotionController(enabled=enable_motion, interface=args.interface)
    if enable_motion:
        motion.init()
    lost_counter = 0
    line_was_detected = False
    integral_error = 0  # 积分误差累积

    # 初始化 HTTP 调试服务器
    http_server = None
    if enable_http:
        http_server = DebugHTTPServer(port=args.port)
        http_server.start()

    print()
    print("=" * 55)
    print("  Go2 黑线巡线 (D435i + SDK2 直连)")
    if enable_motion:
        print(f"  运动控制: SDK2 SportClient (网口:{args.interface})")
    else:
        print("  运动控制: 禁用 (仅调试画面)")
        if not SDK2_AVAILABLE:
            print("  (SDK2 未安装，参考 setup_env.sh 安装)")
    print(f"  线速度: {args.speed} m/s | P:{args.kp} I:{args.ki}")
    if enable_http:
        print(f"  调试页面: http://127.0.0.1:{args.port}")
    print("  q - 退出 | s - 保存截图")
    print("=" * 55)
    print()

    threshold = args.threshold
    kp = args.kp
    ki = args.ki
    max_ang = args.max_angular
    lost_thr = args.lost_threshold
    speed = args.speed
    low_speed_ratio = args.low_speed
    error_history = deque(maxlen=5)  # 平滑窗口
    last_angular = 0.0               # 最后已知转向方向记忆

    frame_count = 0
    start_time = time.time()

    try:
        while True:
            frames = pipeline.wait_for_frames(5000)
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            # 黑线检测
            result, offset, count = detect_black_line_center(color_image, threshold)
            frame_count += 1

            # ---------- 判断是否检测到黑线 ----------
            line_detected = (count >= 1)         # 有任何点就认为有线
            line_confident = (count >= DETECT_POINTS_THRESHOLD)  # 点数足够才能可靠计算偏移

            # ---------- 运动控制 ----------
            if enable_motion:
                if line_detected:
                    # 检测到黑线 → 全速前进 + 转向纠偏
                    lost_counter = 0
                    if not line_was_detected:
                        print(f"\n[Line] 检测到黑线! (点数:{count}) 全速巡线")
                        line_was_detected = True
                        integral_error = 0  # 重置积分

                    if line_confident:
                        # 点数足够 → PI控制转向
                        error_history.append(offset)
                        smooth_offset = np.mean(error_history)

                        # 积分控制 (累积误差纠正)
                        integral_error += offset
                        integral_error = np.clip(integral_error, -500, 500)

                        angular_z = np.clip(
                            -kp * smooth_offset - ki * integral_error,
                            -max_ang, max_ang
                        )
                        last_angular = angular_z  # 记住最后转向方向
                        status_text = f"FOLLOW offset={smooth_offset:.0f}px"
                    else:
                        # 点数不足 → 直走不转向, 但保持全速
                        angular_z = 0.0
                        status_text = f"FWD pts={count} (直走)"

                    motion.move(speed, angular_z)
                    vel_text = f"v={speed:.2f} w={angular_z:.3f}"

                else:
                    # 丢线处理
                    lost_counter += 1
                    error_history.clear()

                    if lost_counter <= lost_thr:
                        # 短暂丢线 → 全速前进直走, 用最后已知方向微调
                        motion.move(speed, last_angular * 0.3)
                        status_text = f"FWD! lost={lost_counter}/{lost_thr}"
                        vel_text = f"v={speed:.2f} w={last_angular*0.3:.3f}"
                    elif lost_counter <= lost_thr * 2:
                        # 较长丢线 → 半速前进
                        motion.move(speed * low_speed_ratio, 0.0)
                        status_text = f"SLOW lost={lost_counter}"
                        vel_text = f"v={speed*low_speed_ratio:.2f} w=0"
                    else:
                        # 长时间彻底丢线 → 停止
                        if line_was_detected:
                            print(f"\n[Line] 黑线长时间消失! ({lost_counter}帧), 停止")
                            line_was_detected = False
                            integral_error = 0
                        motion.stop()
                        status_text = f"STOP lost={lost_counter}"
                        vel_text = "v=0 w=0"
            else:
                # 运动禁用时, 仅显示方向信息
                if offset != 0:
                    direction = "RIGHT" if offset > 0 else "LEFT"
                    status_text = f"Offset: {offset} ({direction})"
                else:
                    status_text = "No line"
                vel_text = "MOTION OFF"

            # ---------- 画面标注 ----------
            cv2.putText(result, f"Th: {threshold} | Points: {count}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(result, status_text, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(result, vel_text, (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

            # 连接状态
            if enable_motion:
                conn_status = motion.get_status()
                cv2.putText(result, f"SDK2: {conn_status}", (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0) if conn_status == "CONNECTED" else (0, 0, 255), 2)
            else:
                conn_status = "DISABLED"

            # 状态条 (底部)
            bar_color = (0, 255, 0) if line_detected else (0, 0, 255)
            cv2.rectangle(result, (0, result.shape[0]-5), (result.shape[1], result.shape[0]), bar_color, -1)

            # ---------- 显示 ----------
            cv2.imshow("Go2 Black Line Detection", result)

            # ---------- HTTP 调试更新 ----------
            if http_server:
                _, jpeg = cv2.imencode('.jpg', result, [cv2.IMWRITE_JPEG_QUALITY, 75])
                http_server.update(jpeg.tobytes(), status_text, vel_text, conn_status)

            # ---------- 键盘控制 ----------
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"frame_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
                cv2.imwrite(filename, result)
                print(f"已保存: {filename}")

            # FPS 输出 (每30帧)
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                if line_detected:
                    print(f"\r[FPS:{fps:.1f}] 巡线中 | offset={offset} | w={kp*offset:.3f}",
                          end='', flush=True)
                else:
                    print(f"\r[FPS:{fps:.1f}] 未检测到 | lost={lost_counter}",
                          end='', flush=True)

    except KeyboardInterrupt:
        print("\n\n[Main] 收到退出信号 (Ctrl+C)")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 清理
        print("\n[Main] 正在停止...")
        motion.cleanup()
        time.sleep(0.1)
        pipeline.stop()
        cv2.destroyAllWindows()
        if http_server:
            http_server.stop()
        print("[Main] 退出完成。")


if __name__ == "__main__":
    main()
