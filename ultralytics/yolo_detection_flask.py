import os

from flask import Flask, request, jsonify, Response
import cv2
import time
import threading
import requests
from ultralytics import YOLO
from functools import wraps
import traceback

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
app = Flask(__name__)

# === 配置参数 ===
MODEL_NAME = "yolo11n.pt"
# 请确保这是正确的 RTSP 地址
RTSP_URL = "rtsp://admin:Zswbimvr@192.168.1.201:554/Streaming/Channels/101"
DOG_CONTROL_URL = "http://127.0.0.1:5007"  # 机器狗控制服务地址
PAUSE_DURATION = 61  # 冷却时间改为 20 秒

# === 全局实例 ===
detector = None
detector_lock = threading.Lock()

# === 全局视频帧缓存 (用于推流) ===
global_frame = None
frame_lock = threading.Lock()


class LatestFrameReader:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src)
        # --- 新增：设置缓冲区大小为1，减少积压 ---
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.lock = threading.Lock()
        self.frame = None
        self.ret = False
        self.running = True
        self.t = threading.Thread(target=self._reader)
        self.t.daemon = True
        self.t.start()

    def _reader(self):
        while self.running:
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.01) # 稍微休眠避免死循环空转
                    continue
                with self.lock:
                    self.ret = ret
                    self.frame = frame # 总是覆盖为最新帧
            else:
                time.sleep(0.1)

    def read(self):
        with self.lock:
            return self.ret, self.frame

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.running = False
        if self.t:
            self.t.join(timeout=1)
        self.cap.release()


class PersonDetector:
    def __init__(self, model_name, rtsp_url, dog_url, pause_duration):
        self.model_name = model_name
        self.rtsp_url = rtsp_url
        self.dog_control_url = dog_url
        self.pause_duration = pause_duration

        self.is_running = False
        self.detection_thread = None
        self.model = None
        self.cap = None

        self.last_detection_time = 0
        self.total_detections = 0
        self.start_time = 0

    def initialize_model(self):
        try:
            print(f"正在加载模型 {self.model_name}...")
            self.model = YOLO(self.model_name)
            print("✅ 模型加载成功")
            return True
        except Exception as e:
            print(f"❌ 模型加载失败: {e}")
            return False

    def open_video_stream(self):
        try:
            print(f"正在打开视频流 {self.rtsp_url} (TCP模式)...")
            # 使用自定义的 LatestFrameReader 替代原生 VideoCapture
            self.cap = LatestFrameReader(self.rtsp_url)

            # 给一点时间让子线程读到第一帧
            time.sleep(1.0)

            if not self.cap.isOpened():
                print("❌ 无法打开视频流")
                return False
            print("✅ 视频流已连接 (低延迟模式)")
            return True
        except Exception as e:
            print(f"❌ 视频流连接失败: {e}")
            return False


    def send_signal_to_dog(self):
        """发送自动打招呼信号"""
        try:
            payload = {
                "action": "greet_auto",
                "reason": "person_detected",
                "timestamp": time.time()
            }
            url = f"{self.dog_control_url}/dog/action"
            print(f"🚨 检测到人员，发送自动打招呼请求...")

            # 【修改】超时时间改为 0.5s，因为我们稍后会修改服务端让其立即返回
            # 即使超时也不要在意，我们只负责通知
            try:
                requests.post(url, json=payload, timeout=0.5)
            except requests.exceptions.ReadTimeout:
                # 这是预期的，如果服务端处理慢，我们不等待
                pass
            except Exception as e:
                print(f"⚠️ 发送请求异常(非致命): {e}")

        except Exception as e:
            print(f"❌ 无法联系机器狗服务: {e}")

    def detect_person(self, results):
        for result in results:
            for box in result.boxes:
                if int(box.cls) == 0:  # class 0 is person
                    return True
        return False

    def detection_loop(self):
        global global_frame
        print("🔍 检测循环已启动")

        # 帧计数器
        frame_count = 0
        # 检测间隔（每隔 3 帧检测一次，根据你的GPU性能调整，性能差就设大点）
        detect_interval = 3

        try:
            while self.is_running:
                if self.cap is None or not self.cap.isOpened():
                    time.sleep(1)
                    continue

                # 1. 获取最新帧
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    time.sleep(0.01)
                    continue

                # 2. 决定是否进行检测
                frame_count += 1
                annotated_frame = frame # 默认就是原图

                # 只有在特定间隔才进行 YOLO 推理
                if frame_count % detect_interval == 0:
                    # 复制一份用于处理，避免影响原图
                    process_frame = frame.copy()

                    # 执行检测
                    results = self.model(process_frame, verbose=False)
                    annotated_frame = results[0].plot() # 画框后的图

                    # 触发打招呼逻辑
                    current_time = time.time()
                    if current_time - self.last_detection_time > self.pause_duration:
                        if self.detect_person(results):
                            print(f"👤 检测到人员！触发交互")
                            self.last_detection_time = current_time
                            self.total_detections += 1
                            threading.Thread(target=self.send_signal_to_dog, daemon=True).start()

                # 3. 更新全局帧 (这里非常关键：无论是否检测，都更新画面)
                # 注意：如果跳帧检测，非检测帧将没有框。
                # 如果希望一直有框，需要缓存上一次的 results 并重复画上去，这里为了低延迟先只显示最新画面
                with frame_lock:
                    global_frame = annotated_frame.copy()

                time.sleep(0.005)

        except Exception as e:
            print(f"❌ 检测循环异常: {e}")
            traceback.print_exc()
        finally:
            if self.cap:
                self.cap.release()

    def start_detection(self):
        if self.is_running:
            return {"success": False, "message": "检测已经在运行中"}

        if not self.model:
            if not self.initialize_model():
                return {"success": False, "message": "模型初始化失败"}

        if not self.open_video_stream():
            return {"success": False, "message": "视频流连接失败"}

        self.is_running = True
        self.start_time = time.time()
        self.detection_thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.detection_thread.start()

        return {"success": True, "message": "人员检测已启动"}
    
    def stop_detection(self):
        """停止检测"""
        if not self.is_running:
            return {"success": False, "message": "检测未在运行"}
        
        self.is_running = False
        if self.detection_thread:
            self.detection_thread.join(timeout=3)
        
        if self.cap:
            self.cap.release()
            self.cap = None
        
        return {"success": True, "message": "人员检测已停止"}
    
    def get_status(self):
        """获取检测状态"""
        uptime = time.time() - self.start_time if self.is_running else 0
        return {
            "is_running": self.is_running,
            "total_detections": self.total_detections,
            "uptime_seconds": round(uptime, 2),
            "last_detection_ago": round(time.time() - self.last_detection_time, 2) if self.last_detection_time > 0 else None
        }


# ==========================================
# 2. Flask 辅助函数
# ==========================================
def get_detector():
    """获取或初始化检测器实例"""
    global detector
    with detector_lock:
        if detector is None:
            detector = PersonDetector(
                model_name=MODEL_NAME,
                rtsp_url=RTSP_URL,
                dog_url=DOG_CONTROL_URL,
                pause_duration=PAUSE_DURATION
            )
        return detector


def require_detector(f):
    """装饰器：确保检测器已初始化"""
    @wraps(f)
    def decorated_function(*args, **kwargs):
        det = get_detector()
        if not det:
            return jsonify({"success": False, "message": "检测器未初始化"}), 500
        try:
            return f(det, *args, **kwargs)
        except Exception as e:
            print(f"❌ 执行失败: {e}")
            traceback.print_exc()
            return jsonify({"success": False, "message": str(e)}), 500
    return decorated_function


def generate_frames():
    global global_frame
    # 记录上一帧的时间戳，用于控制最大帧率，而不是强制 sleep
    last_time = 0
    target_fps = 30
    frame_interval = 1.0 / target_fps

    while True:
        current_time = time.time()
        # 如果距离上一帧时间太短，就跳过，避免发送太快浏览器处理不过来
        if current_time - last_time < frame_interval:
            time.sleep(0.001) # 极短休眠释放 CPU
            continue

        frame_to_encode = None
        with frame_lock:
            if global_frame is None:
                time.sleep(0.01)
                continue
            frame_to_encode = global_frame # 这里其实不需要 copy，因为 imencode 很快且 global_frame 会被整体替换

        if frame_to_encode is not None:
            try:
                # 降低 JPEG 质量以减少数据量和编码时间 (质量 0-100，默认 95)
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
                ret, buffer = cv2.imencode('.jpg', frame_to_encode, encode_param)

                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    last_time = time.time()
            except Exception as e:
                print(f"编码错误: {e}")


# ==========================================
# 3. Flask 路由接口
# ==========================================

@app.route('/video_feed')
def video_feed():
    """前端 <img> 标签的 src 地址"""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/detection/start', methods=['POST'])
@require_detector
def start_detection(det):
    """启动人员检测"""
    result = det.start_detection()
    status_code = 200 if result["success"] else 400
    return jsonify(result), status_code


@app.route('/detection/stop', methods=['POST'])
@require_detector
def stop_detection(det):
    """停止人员检测"""
    result = det.stop_detection()
    status_code = 200 if result["success"] else 400
    return jsonify(result), status_code


@app.route('/detection/status', methods=['GET'])
@require_detector
def detection_status(det):
    """获取检测状态"""
    status = det.get_status()
    return jsonify({"success": True, "status": status})


@app.route('/detection/config', methods=['GET', 'POST'])
@require_detector
def detection_config(det):
    """获取或更新配置"""
    if request.method == 'GET':
        config = {
            "model_name": det.model_name,
            "rtsp_url": det.rtsp_url,
            "dog_control_url": det.dog_control_url,
            "pause_duration": det.pause_duration
        }
        return jsonify({"success": True, "config": config})
    
    elif request.method == 'POST':
        if det.is_running:
            return jsonify({"success": False, "message": "请先停止检测再修改配置"}), 400
        
        data = request.json
        if "pause_duration" in data:
            det.pause_duration = float(data["pause_duration"])
        if "dog_control_url" in data:
            det.dog_control_url = data["dog_control_url"]
        
        return jsonify({"success": True, "message": "配置已更新"})


@app.route('/health', methods=['GET'])
def health_check():
    """健康检查接口"""
    return jsonify({
        "success": True,
        "service": "person_detection",
        "status": "running"
    })

@app.route('/detection/start', methods=['POST'])
@require_detector
def start_detection_route(det):
    return jsonify(det.start_detection())


if __name__ == "__main__":
    print(" 启动人员检测服务 on port 5008...")

    # 1. 手动初始化检测器
    det = get_detector()

    # 2. 自动启动检测线程 (这样一运行py文件，摄像头就开始工作)
    print("正在自动启动检测线程...")
    start_result = det.start_detection()
    print(f"自动启动结果: {start_result}")

    # 3. 启动 Flask
    app.run(host='0.0.0.0', port=5008, threaded=True)
