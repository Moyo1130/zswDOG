from flask import Flask, request, jsonify
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
RTSP_URL = "rtsp://admin:Zswbimvr@192.168.1.201:554/Streaming/Channels/101"
DOG_CONTROL_URL = "http://127.0.0.1:5007"  # 机器狗控制服务地址
PAUSE_DURATION = 10  # 检测到人后暂停检测的秒数

# === 全局实例 ===
detector = None
detector_lock = threading.Lock()


# ==========================================
# 1. 人员检测管理器类
# ==========================================
class PersonDetector:
    def __init__(self, model_name, rtsp_url, dog_url, pause_duration):
        self.model_name = model_name
        self.rtsp_url = rtsp_url
        self.dog_control_url = dog_url
        self.pause_duration = pause_duration
        
        # 状态管理
        self.is_running = False
        self.detection_thread = None
        self.model = None
        self.cap = None
        
        # 统计信息
        self.last_detection_time = 0
        self.total_detections = 0
        self.start_time = 0
        
    def initialize_model(self):
        """初始化 YOLO 模型"""
        try:
            print(f"正在加载模型 {self.model_name}...")
            self.model = YOLO(self.model_name)
            print("✅ 模型加载成功")
            return True
        except Exception as e:
            print(f"❌ 模型加载失败: {e}")
            return False
    
    def open_video_stream(self):
        """打开视频流"""
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
    
    def send_signal_to_dog(self, object_type):
        """向机器狗服务发送控制信号
        
        Args:
            object_type: 检测到的目标类型，如 "person", "car", "dog" 等
        """
        # 目标类型到动作和原因的映射表（可扩展）
        object_action_map = {
            "person": {
                "action": "greet",
                "reason": "person_detected",
                "description": "检测到人员，发送打招呼信号"
            },
            # 未来可扩展其他目标类型
            # "car": {
            #     "action": "avoid",
            #     "reason": "car_detected",
            #     "description": "检测到车辆，发送避让信号"
            # },
        }
        
        # 获取对应的动作配置，如果不存在则使用默认停止动作
        action_config = object_action_map.get(object_type, {
            "action": "stop",
            "reason": f"{object_type}_detected",
            "description": f"检测到{object_type}，发送停止信号"
        })
        
        try:
            payload = {
                "action": action_config["action"],
                "reason": action_config["reason"],
                "timestamp": time.time(),
                "detected_object": object_type
            }
            url = f"{self.dog_control_url}/dog/action"
            print(f"🚨 {action_config['description']}...")
            
            response = requests.post(url, json=payload, timeout=5)
            if response.status_code == 200:
                print(f"✅ 机器狗响应成功: {response.json()}")
                return True
            else:
                print(f"⚠️ 机器狗响应异常: {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            print(f"❌ 无法联系机器狗服务: {e}")
            return False
    
    def detect_person(self, results):
        """检测结果中是否包含人物
        
        Args:
            results: YOLO 模型的检测结果
            
        Returns:
            bool: 如果检测到人返回 True，否则返回 False
        """
        for result in results:
            for box in result.boxes:
                if int(box.cls) == 0:  # class 0 是人 (COCO dataset)
                    return True
        return False
    
    def detection_loop(self):
        """检测主循环"""
        print("🔍 检测循环已启动")
        
        try:
            while self.is_running:
                ret, frame = self.cap.read()
                if not ret:
                    print("⚠️ 读取帧失败，尝试重连...")
                    time.sleep(1)
                    if not self.open_video_stream():
                        break
                    continue
                
                current_time = time.time()
                time_since_last_detection = current_time - self.last_detection_time
                
                # 检查是否在冷却期
                if time_since_last_detection > self.pause_duration:
                    # 执行 YOLO 推理
                    results = self.model(frame, verbose=False)
                    
                    # 调用人物检测函数判断是否有人
                    if self.detect_person(results):
                        print(f"👤 检测到人员！暂停检测 {self.pause_duration} 秒")
                        self.last_detection_time = current_time
                        self.total_detections += 1
                        
                        # 异步发送信号给机器狗，传入目标类型 "person"
                        threading.Thread(
                            target=self.send_signal_to_dog,
                            args=("person",),
                            daemon=True
                        ).start()
                
                # 控制帧率，避免过度消耗CPU
                time.sleep(0.03)  # 约30fps
        
        except Exception as e:
            print(f"❌ 检测循环异常: {e}")
            traceback.print_exc()
        finally:
            if self.cap:
                self.cap.release()
                print("📹 视频流已释放")
    
    def start_detection(self):
        """启动检测"""
        if self.is_running:
            return {"success": False, "message": "检测已经在运行中"}
        
        # 初始化模型
        if not self.model:
            if not self.initialize_model():
                return {"success": False, "message": "模型初始化失败"}
        
        # 打开视频流
        if not self.open_video_stream():
            return {"success": False, "message": "视频流连接失败"}
        
        # 启动检测线程
        self.is_running = True
        self.start_time = time.time()
        self.total_detections = 0
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


# ==========================================
# 3. Flask 路由接口
# ==========================================
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


if __name__ == "__main__":
    print("🚀 启动人员检测服务 on port 5008...")
    app.run(host='0.0.0.0', port=5008, threaded=True)
