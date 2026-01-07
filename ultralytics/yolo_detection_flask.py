from flask import Flask, request, jsonify, Response
import cv2
import time
import threading
import requests
from ultralytics import YOLO
from functools import wraps
import traceback

app = Flask(__name__)

# === 配置参数 ===
MODEL_NAME = "yolo11n.pt"
# 请确保这是正确的 RTSP 地址
RTSP_URL = "rtsp://admin:Zswbimvr@192.168.1.201:554/Streaming/Channels/101"
DOG_CONTROL_URL = "http://127.0.0.1:5007"  # 机器狗控制服务地址
PAUSE_DURATION = 20  # 【修改】冷却时间改为 20 秒

# === 全局实例 ===
detector = None
detector_lock = threading.Lock()

# === 全局视频帧缓存 (用于推流) ===
global_frame = None
frame_lock = threading.Lock()

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
            print(f"正在打开视频流 {self.rtsp_url}...")
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if not self.cap.isOpened():
                print("❌ 无法打开视频流")
                return False
            print("✅ 视频流已连接")
            return True
        except Exception as e:
            print(f"❌ 视频流连接失败: {e}")
            return False

    def send_signal_to_dog(self):
        """发送自动打招呼信号"""
        try:
            # 【修改】发送特殊的 greet_auto 动作
            payload = {
                "action": "greet_auto",
                "reason": "person_detected",
                "timestamp": time.time()
            }
            url = f"{self.dog_control_url}/dog/action"
            print(f"🚨 检测到人员，发送自动打招呼请求...")

            # 使用极短的超时，避免阻塞检测线程
            requests.post(url, json=payload, timeout=2)
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

        try:
            while self.is_running:
                if self.cap is None or not self.cap.isOpened():
                    time.sleep(1)
                    continue

                ret, frame = self.cap.read()
                if not ret:
                    print("⚠️ 读取帧失败，尝试重连...")
                    self.cap.release()
                    time.sleep(1)
                    self.open_video_stream()
                    continue

                # 1. 执行检测
                current_time = time.time()
                annotated_frame = frame # 默认显示原图

                # 只有在非冷却期才进行推理，节省资源，或者一直推理但只在非冷却期触发动作
                # 这里选择一直推理以便在前端画框
                results = self.model(frame, verbose=False)
                annotated_frame = results[0].plot() # 画框

                # 2. 触发逻辑
                time_since_last = current_time - self.last_detection_time

                if time_since_last > self.pause_duration:
                    if self.detect_person(results):
                        print(f"👤 检测到人员！触发交互 (冷却 {self.pause_duration}s)")
                        self.last_detection_time = current_time
                        self.total_detections += 1

                        # 异步发送信号
                        threading.Thread(target=self.send_signal_to_dog, daemon=True).start()

                # 3. 更新全局帧供推流使用
                with frame_lock:
                    global_frame = annotated_frame.copy()

                # 控制帧率
                time.sleep(0.03)

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
    while True:
        with frame_lock:
            if global_frame is None:
                time.sleep(0.1)
                continue

            # 编码为 JPEG
            ret, buffer = cv2.imencode('.jpg', global_frame)
            frame_bytes = buffer.tobytes()

        # 生成 MJPEG 流格式
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.04) # 限制推流帧率约 25fps


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
    app.run(host='0.0.0.0', port=5008, threaded=True)
