from flask import Flask, request, jsonify
from totalController import Controller
import threading
import time
import math
from functools import wraps
from collections import deque

# === ROS 依赖 ===
try:
    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped

    ROS_AVAILABLE = True
except ImportError:
    print("⚠️ 警告: 未检测到 rospy 环境，巡逻功能将不可用")
    ROS_AVAILABLE = False

app = Flask(__name__)

# === 机器狗内部配置 ===
# 这里通常是机器狗内部主板的IP，保持默认即可
ROBOT_IP = "192.168.1.120"
ROBOT_PORT = 43893  # 或者是 43893，根据 totalController 确定

# === 全局实例 ===
robot = None  # 基础控制器实例
patrol_manager = None  # 巡逻管理器实例
recorder = None
robot_lock = threading.Lock()


# 角度归一化
def normalize_angle(angle):
    """将角度归一化到 -pi ~ pi 之间"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# ==========================================
# 1. 巡逻管理器类 (AutoPatrolController)
# ==========================================
class AutoPatrolController:
    def __init__(self, controller_instance):
        self.controller = controller_instance

        # 坐标系相关
        self.raw_pose = (0.0, 0.0, 0.0)
        self.origin_pose = None
        self.latest_pose = (0.0, 0.0, 0.0)

        # 路径与状态
        self.waypoints_queue = deque()
        self.current_target = None
        self.is_patrolling = False
        self.running = True
        self.is_paused = False  # 暂停标志位

        # 卡点保护 / 状态计时器
        self.target_start_time = 0
        self.close_proximity_start_time = None # [新增] 进入目标附近的时刻

        self.patrol_mode = "ONE_WAY"
        self.original_path = []
        self.current_path_points = []

        # === PID 参数 ===
        self.kp_linear = 0.6
        self.kp_angular = 1.1          # 稍微加强一点转向力度
        self.min_physical_speed = 0.15 # 稍微降低最小启动速度
        self.max_linear_speed = 0.6
        self.max_angular_speed = 0.8
        self.arrival_threshold = 0.20  # [修改] 到达阈值放宽到 20cm

        if ROS_AVAILABLE:
            if rospy.get_node_uri() is None:
                rospy.init_node('dog_flask_merged_node', anonymous=True, disable_signals=True)
            rospy.Subscriber("/leg_odom", PoseWithCovarianceStamped, self.pose_callback)
            threading.Thread(target=self.control_loop, daemon=True).start()

    # 巡逻暂停
    def pause(self):
        if self.is_patrolling and not self.is_paused:
            print("⏸️ 巡逻已暂停 (保持当前目标)")
            self.is_paused = True
            # 发送停止指令给底层
            self.controller.move(0, 0, 0)
            self.controller.stop_continuous_move()

    # 巡逻恢复
    def resume(self):
        if self.is_patrolling and self.is_paused:
            print("▶️ 巡逻已恢复")
            self.is_paused = False
            # 这里的 control_loop 会自动接管，不需要额外操作


    def reset_origin(self):
        if self.raw_pose != (0.0, 0.0, 0.0):
            self.origin_pose = self.raw_pose
            self.latest_pose = (0.0, 0.0, 0.0) # 重置为原点
            print(f"✅ 全局坐标系已建立/重置 (Raw Origin: {self.origin_pose})")
        else:
            print("⚠️ 警告: 尚未收到odom数据，无法重置原点")

    def _calculate_local_offset(self, current_global, next_node_global):
        cx, cy, cyaw = current_global
        nx, ny, nyaw = next_node_global if len(next_node_global) == 3 else (
        next_node_global[0], next_node_global[1], 0.0)

        # 1. 计算全局坐标差
        dx = nx - cx
        dy = ny - cy

        # 2. 旋转平移量 (全局delta -> 局部delta)
        # 我们需要将向量 (dx, dy) 逆时针旋转 -cyaw 度
        cos_val = math.cos(-cyaw)
        sin_val = math.sin(-cyaw)

        local_x = dx * cos_val - dy * sin_val
        local_y = dx * sin_val + dy * cos_val

        # 3. 计算角度差
        local_yaw = normalize_angle(nyaw - cyaw)

        return (local_x, local_y, local_yaw)

    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.raw_pose = (pos.x, pos.y, yaw)

        # 实时计算相对于原点的全局坐标
        if self.origin_pose:
            self.latest_pose = transform_to_local(self.raw_pose, self.origin_pose)
        else:
            self.latest_pose = self.raw_pose

    def start_patrol(self, points, mode="ONE_WAY"):
        self.stop_patrol()
        with threading.Lock():
            self.original_path = list(points)
            self.patrol_mode = mode
            self.current_path_points = points

            # === 修改 1: 仅在开始巡逻时重置一次原点 ===
            print("🏁 初始化巡逻，重置坐标原点...")
            self.reset_origin()
            time.sleep(0.2) # 等待odom刷新

            # 寻找最近点（如果没有指定从头开始）
            # 注意：这里的最近点逻辑基于假设刚reset后我们在(0,0)附近
            closest_idx = 0
            # 如果需要智能寻找最近点，需要遍历points计算距离(0,0)最近的点
            # 这里简化为从第0个点开始，或者你也可以保留原来的逻辑

            start_points = points[closest_idx:]
            for p in start_points:
                self._append_to_queue(p)

            self.is_patrolling = True
            print(f"🚀 巡逻开始 | 模式: {mode} | 全局参考系模式")

    def _append_to_queue(self, p):
        if len(p) == 2:
            self.waypoints_queue.append((float(p[0]), float(p[1]), 0.0))
        else:
            self.waypoints_queue.append((float(p[0]), float(p[1]), float(p[2])))

    def _find_closest_point_index(self, points):
        curr_x, curr_y, _ = self.latest_pose
        min_dist = float('inf')
        min_idx = 0
        for i, p in enumerate(points):
            dist = math.sqrt((p[0] - curr_x) ** 2 + (p[1] - curr_y) ** 2)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        return min_idx

    def stop_patrol(self):
        self.is_patrolling = False
        self.waypoints_queue.clear()
        self.current_target = None
        if self.controller:
            try:
                self.controller.move(0, 0, 0)
                self.controller.stop_continuous_move()
                time.sleep(0.1)
                self.controller.move(0, 0, 0)
            except Exception as e:
                print(f"停止指令异常: {e}")

    def _apply_deadzone(self, value, deadzone=0.2, max_val=1.0):
        if abs(value) < 0.05: return 0.0
        sign = 1.0 if value > 0 else -1.0
        abs_val = abs(value)
        mapped_val = deadzone + (max_val - deadzone) * abs_val
        final_val = min(mapped_val, max_val)
        return sign * final_val

    def _calculate_required_turn_degrees(self, current_pose, target_point):
        """
        计算从当前位置可以直接面向目标点所需的旋转角度（度）
        根据你的需求，这会解决 0 -> -1.57 (右转90) 或 -1.57 -> 0.86 (左转135) 的逻辑
        """
        curr_x, curr_y, curr_yaw = current_pose
        tgt_x, tgt_y, _ = target_point  # 我们只关心去哪里的坐标，不关心终点朝向

        # 1. 计算去往目标的几何向量
        dx = tgt_x - curr_x
        dy = tgt_y - curr_y

        # 距离如果太近（小于5cm），不需要转向，直接前进处理
        if math.sqrt(dx * dx + dy * dy) < 0.05:
            return 0.0

        # 2. 计算目标向量的绝对朝向 (atan2 返回 -pi ~ pi)
        # 例如：目标在正右边，角度是0；在正前方(x轴)，角度是0?
        # 注意：ROS/数学坐标系通常 X轴朝前为0，Y轴朝左为正90(1.57)
        # 你的坐标：[11.720, 0.010] -> [11.790, -5.019]
        # dx=0.07, dy=-5.029. atan2(-5, 0.07) ≈ -1.55 (即 -89度，右边)
        target_vector_angle = math.atan2(dy, dx)

        # 3. 计算差异：我要去的方向 - 我现在的朝向
        angle_diff = target_vector_angle - curr_yaw
        angle_diff = normalize_angle(angle_diff)  # 归一化到 -pi ~ pi

        return math.degrees(angle_diff)

    def control_loop(self):
        rate = 20
        dt = 1.0 / rate

        # 记录每段路径的最小剩余距离，防止错过点后一直跑
        min_dist_record = float('inf')
        last_log_time = 0

        while self.running:
            try:
                if self.is_paused:
                    time.sleep(0.2)
                    continue
                # 状态检查
                if not self.is_patrolling or not ROS_AVAILABLE or self.latest_pose is None:
                    time.sleep(0.2)
                    continue

                curr_x, curr_y, curr_yaw = self.latest_pose

                # ----------------------------------------------------
                # 阶段 1：获取新目标
                # ----------------------------------------------------
                if self.current_target is None:
                    if len(self.waypoints_queue) > 0:
                        next_global = self.waypoints_queue[0]
                        self.current_target = next_global
                        self.target_start_time = time.time()
                        self.close_proximity_start_time = None
                        min_dist_record = float('inf')  # 重置最小距离记录

                        target_x, target_y, _ = next_global
                        print(
                            f"\n📍 [NEW GOAL] 前往新航点: ({target_x:.2f}, {target_y:.2f}) | 当前位置: ({curr_x:.2f}, {curr_y:.2f})")

                        # 初始大角度转向逻辑
                        dx = target_x - curr_x
                        dy = target_y - curr_y
                        dist = math.sqrt(dx ** 2 + dy ** 2)
                        desired_global_yaw = math.atan2(dy, dx)
                        angle_diff = normalize_angle(desired_global_yaw - curr_yaw)
                        turn_deg = math.degrees(angle_diff)

                        if dist > 0.10 and abs(turn_deg) > 20.0:  # 加大角度阈值
                            print(f"👉 执行初始转向修正: {turn_deg:.1f}°")
                            execute_precise_turn(self.controller, self, turn_deg)
                            time.sleep(0.5)
                            # 转向完重新获取位置
                            continue
                    else:
                        # 队列处理
                        if self.patrol_mode == "LOOP":
                            print("🔄 循环模式：重新加载所有点")
                            for p in self.original_path:
                                self._append_to_queue(p)
                            continue
                        else:
                            print("✅ 巡逻任务完成")
                            self.stop_patrol()
                            continue

                # ----------------------------------------------------
                # 阶段 2：PID 控制与到达判定
                # ----------------------------------------------------
                tgt_x, tgt_y, _ = self.current_target

                # 1. 计算全局误差
                global_err_x = tgt_x - curr_x
                global_err_y = tgt_y - curr_y
                dist_remaining = math.sqrt(global_err_x ** 2 + global_err_y ** 2)

                # 更新最小距离记录
                if dist_remaining < min_dist_record:
                    min_dist_record = dist_remaining

                # ===== 日志打印 (每 1.5 秒打印一次) =====
                if time.time() - last_log_time > 1.5:
                    print(
                        f"DEBUG: 坐标({curr_x:.2f},{curr_y:.2f})->目标({tgt_x:.2f},{tgt_y:.2f}) | 距离: {dist_remaining:.2f}m")
                    last_log_time = time.time()

                # ===== 判定逻辑 =====
                is_arrived = False

                # 判定 A: 距离达标
                if dist_remaining < self.arrival_threshold:
                    print(f"✅ 到达航点 (距离触发) | 剩余: {dist_remaining:.3f}m")
                    is_arrived = True

                # 判定 B: 近距离防震荡
                if dist_remaining < 0.50:
                    if self.close_proximity_start_time is None:
                        self.close_proximity_start_time = time.time()
                    elif time.time() - self.close_proximity_start_time > 3.0:
                        print(f"⚠️ 接近目标超时 (防震荡触发) | 剩余: {dist_remaining:.3f}m")
                        is_arrived = True
                else:
                    self.close_proximity_start_time = None

                # 判定 C: 越过目标太远自动放弃 (比如错过了 1.5 米)
                if dist_remaining > min_dist_record + 1.5 and min_dist_record < 5.0:
                    print(f"⏭️ 似乎已越过目标并远离 ({dist_remaining:.2f}m > min {min_dist_record:.2f}m)，强制判定到达")
                    is_arrived = True

                # 判定 D: 总超时
                if time.time() - self.target_start_time > 180.0:  # 60秒还没走到一个点
                    print("⌛ 单点耗时过长，强制跳过")
                    is_arrived = True

                if is_arrived:
                    self.controller.start_continuous_move(0, 0, 0)
                    self.waypoints_queue.popleft()
                    self.current_target = None
                    min_dist_record = float('inf')  # 重置
                    continue
                # ===============================================

                # 2. 坐标转换 (Global -> Body)
                cos_yaw = math.cos(curr_yaw)
                sin_yaw = math.sin(curr_yaw)

                body_err_x = cos_yaw * global_err_x + sin_yaw * global_err_y
                body_err_y = -sin_yaw * global_err_x + cos_yaw * global_err_y

                # 3. 计算 PID
                angle_error_local = math.atan2(body_err_y, body_err_x)

                # 距离越远，允许的速度越大；距离近了要减速
                target_speed = self.kp_linear * body_err_x

                # 角度偏差修正逻辑：如果偏得厉害，先别走那么快
                if abs(angle_error_local) > 0.3:  # 大约17度
                    target_speed *= 0.5
                if abs(angle_error_local) > 0.8:  # 大约45度
                    target_speed = 0.0  # 纯旋转

                # 限幅
                v_cmd = max(min(target_speed, self.max_linear_speed), -self.max_linear_speed)

                # 最小速度保持
                if abs(v_cmd) > 0.01 and abs(v_cmd) < self.min_physical_speed:
                    v_cmd = math.copysign(self.min_physical_speed, v_cmd)

                # 禁止倒车 (可选，保持路径跟随稳定性)
                if v_cmd < 0: v_cmd = 0

                w_cmd = self.kp_angular * angle_error_local
                w_cmd = max(min(w_cmd, self.max_angular_speed), -self.max_angular_speed)

                self.controller.start_continuous_move(v_cmd, 0, -w_cmd)

                time.sleep(dt)

            except Exception as e:
                print(f"❌ 控制循环异常: {e}")
                time.sleep(1.0)


    def return_safely(self):
        if not self.original_path:
            self.start_patrol([(0.0, 0.0, 0.0)], mode="ONE_WAY")
            return {"success": True, "message": "无历史路径，直接返回原点"}

        self.stop_patrol()
        closest_idx = self._find_closest_point_index(self.current_path_points)
        path_so_far = self.current_path_points[0: closest_idx + 1]
        return_path = list(reversed(path_so_far))
        return_path.append((0.0, 0.0, 0.0))

        self.start_patrol(return_path, mode="ONE_WAY")
        return {"success": True, "message": "开始沿原路径安全撤退"}


# ==========================================
# 1.5 路径录制管理器 (新增)
# ==========================================
class PathRecorder:
    def __init__(self, patrol_controller):
        self.patrol_ctrl = patrol_controller
        self.recorded_path_global = []  # 存储用于显示的全局路径

        # 累积器：记录从“上一次确认点”到“当前”总共走了多远（全局参考系下）
        # 实际上，因为我们每次录制完都清零 Odom，所以 Odom 的读数就是“当前段的相对位移”
        # 我们只需要维护一个 全局的求和 即可。
        self.current_global_cursor = (0.0, 0.0, 0.0)

    def clear(self):
        self.recorded_path_global = []
        self.current_global_cursor = (0.0, 0.0, 0.0)
        # 清空物理坐标系
        self.patrol_ctrl.reset_origin()
        print("路径录制已清空，里程计已重置")

    def record_current_point(self):
        """
        录制点位：
        1. 读取当前的 Odom (这就是相对于上一个记录点的位移)
        2. 将其叠加到全局游标上，形成全局坐标保存
        3. 【重要】重置 Odom，为下一段录制做准备
        """
        # 1. 获取当前相对于上一次重置后的位移
        local_x, local_y, local_yaw = self.patrol_ctrl.latest_pose

        # 2. 计算这个位移对应的全局新坐标
        # 上一个全局点
        gx, gy, gyaw = self.current_global_cursor

        # 变换公式：将局部增量 (local_x, local_y) 旋转 gyaw 度，加到 (gx, gy) 上
        cos_v = math.cos(gyaw)
        sin_v = math.sin(gyaw)

        new_global_x = gx + (local_x * cos_v - local_y * sin_v)
        new_global_y = gy + (local_x * sin_v + local_y * cos_v)
        new_global_yaw = normalize_angle(gyaw + local_yaw)

        new_point_global = (new_global_x, new_global_y, new_global_yaw)

        # 校验：如果是第一个点，可能就是 (0,0,0) 或者极其接近
        if len(self.recorded_path_global) == 0:
            # 强制第一个点是对齐的，或者直接记录
            pass

        # 3. 保存全局坐标（为了显示给用户看，和之后回放用）
        self.recorded_path_global.append(new_point_global)

        # 更新游标
        self.current_global_cursor = new_point_global

        # 4. 【核心】重置 Odom
        # 这样用户从 A 走到 B，无论中间怎么乱走，只要停在 B 点点录制，
        # 我们记录下 A->B 的向量后，立刻把 B 设为新的 0 点。
        self.patrol_ctrl.reset_origin()
        time.sleep(0.5)

        print(f" 点位已记录(全局): {new_point_global}")
        print(f"   (本段相对位移: {local_x:.2f}, {local_y:.2f}, {local_yaw:.2f})")
        print("   >>> 里程计已重置，请继续前往下一个点")

        return {
            "success": True,
            "message": f"点位已记录。全局坐标: ({new_global_x:.2f}, {new_global_y:.2f})",
            "point": new_point_global,
            "count": len(self.recorded_path_global)
        }

    def get_path_string(self):
        # 返回格式化字符串，直接用于复制到 Java/Prompt
        return "[" + ", ".join([f"[{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]" for p in self.recorded_path_global]) + "]"


# ==========================================
# 2. 基础 Flask 辅助函数
# ==========================================

def get_robot():
    global robot, patrol_manager, recorder
    with robot_lock:
        if robot is None:
            try:
                robot = Controller((ROBOT_IP, ROBOT_PORT))
                robot.start_heartbeat(frequency=2.0)
                patrol_manager = AutoPatrolController(robot)
                recorder = PathRecorder(patrol_manager) # 初始化录制器
                print("机器狗控制器及组件已连接")
            except Exception as e:
                return None
        return robot


def require_robot(f):
    @wraps(f)
    def decorated_function(*args, **kwargs):
        controller = get_robot()
        if not controller:
            return jsonify({"success": False, "message": "控制器未连接"}), 500
        try:
            return f(controller, *args, **kwargs)
        except Exception as e:
            print(f"❌ 执行失败: {e}")
            import traceback
            traceback.print_exc()
            return jsonify({"success": False, "message": str(e)}), 500

    return decorated_function


def transform_to_local(raw_pose, origin_pose):
    """
    将原始 ROS 坐标转换为基于原点 (0,0,0) 的相对坐标
    输入: raw_pose (x, y, yaw), origin_pose (x0, y0, yaw0)
    输出: (x_local, y_local, yaw_local)
    """
    x, y, yaw = raw_pose
    x0, y0, yaw0 = origin_pose

    # 1. 平移
    dx = x - x0
    dy = y - y0

    # 2. 旋转 (逆时针旋转 -yaw0)
    cos_theta = math.cos(-yaw0)
    sin_theta = math.sin(-yaw0)

    x_local = dx * cos_theta - dy * sin_theta
    y_local = dx * sin_theta + dy * cos_theta
    yaw_local = yaw - yaw0

    # 归一化角度到 -pi ~ pi
    while yaw_local > math.pi: yaw_local -= 2 * math.pi
    while yaw_local < -math.pi: yaw_local += 2 * math.pi

    return (x_local, y_local, yaw_local)


# 闭环旋转函数
def execute_precise_turn(controller, patrol_manager, target_angle_degrees, timeout=8.0, post_delay=0.3):
    if not patrol_manager or not ROS_AVAILABLE:
        print(" 无法获取 Odom 数据，无法执行精准转向")
        return False

    _, _, start_yaw = patrol_manager.raw_pose
    radian_delta = math.radians(target_angle_degrees)
    target_yaw = normalize_angle(start_yaw + radian_delta)

    print(f"🔄 精准转向: {target_angle_degrees:.1f}° | TgtYaw: {math.degrees(target_yaw):.1f}°")

    Kp = 1.5
    max_speed = 1.0
    min_speed = 0.2
    tolerance = 0.05 # ~3度

    start_time = time.time()

    try:
        while True:
            if time.time() - start_time > timeout:
                print("❌ 转向超时")
                break

            _, _, current_yaw = patrol_manager.raw_pose
            error = normalize_angle(target_yaw - current_yaw) # 正值=目标在左边

            if abs(error) < tolerance:
                break

            # PID 计算 (正值代表需要向左转)
            turn_speed = Kp * error
            turn_speed = max(min(turn_speed, max_speed), -max_speed)
            if abs(turn_speed) < min_speed:
                turn_speed = math.copysign(min_speed, turn_speed)

            # 【核心修复】数位取反发送给控制器
            # 数学Positive(左) -> Controller Negative(左)
            controller.start_continuous_move(0, 0, -turn_speed)
            time.sleep(0.05)

    except Exception as e:
        print(f"转向异常: {e}")
    finally:
        controller.start_continuous_move(0, 0, 0)
        time.sleep(post_delay)

    return True

# ==========================================
# 3. Flask 路由接口
# ==========================================
@app.route('/dog/init', methods=['POST'])
@require_robot
def init_dog(controller):
    # 获取参数，默认重置坐标，但在连续测试时可以传 False
    data = request.get_json(silent=True) or {}
    should_reset_odom = data.get("reset_odom", True)

    # === 修复：只在非巡逻状态下才重置坐标 ===
    if patrol_manager:
        if patrol_manager.is_patrolling:
            print("⚠️ 机器狗正在巡逻中，无法进行初始化")
            return jsonify({
                "success": False,
                "message": "机器狗正在巡逻中，请先停止巡逻"
            }), 400

        print("初始化被触发...")
        patrol_manager.stop_patrol()
        time.sleep(0.5)

    controller.initialize()
    print("机器人起立...")
    controller.voice_command("STAND")
    time.sleep(4)

    # 只在确实需要时重置原点
    if should_reset_odom and patrol_manager:
        print("正在设置当前位置为绝对原点 (0,0,0)...")
        patrol_manager.reset_origin()
    else:
        print("保持原有坐标系，不重置原点。")

    controller.switch_to_move_mode()
    time.sleep(1)

    return jsonify({"success": True, "message": "初始化完成"})


@app.route('/dog/reset_odom', methods=['POST'])
@require_robot
def reset_odom(controller):
    """
    强制重置坐标系原点为当前位置。
    用于开始新一轮测试循环。
    """
    if patrol_manager:
        patrol_manager.reset_origin()
        return jsonify({"success": True, "message": "坐标系已重置，当前位置为新原点 (0,0,0)"})
    return jsonify({"success": False, "message": "巡逻管理器未初始化"}), 500

@app.route('/dog/record/start', methods=['POST'])
@require_robot
def start_record(controller):
    if recorder:
        recorder.clear()
        # 自动记录起点
        return jsonify(recorder.record_current_point())
    return jsonify({"success": False, "message": "Recorder未初始化"})

@app.route('/dog/record/point', methods=['POST'])
@require_robot
def record_point(controller):
    if recorder:
        return jsonify(recorder.record_current_point())
    return jsonify({"success": False, "message": "Recorder未初始化"})

@app.route('/dog/record/undo', methods=['POST'])
@require_robot
def undo_point(controller):
    if recorder:
        return jsonify(recorder.undo_last_point())
    return jsonify({"success": False, "message": "Recorder未初始化"})

@app.route('/dog/record/return', methods=['POST'])
@require_robot
def return_home(controller):
    if recorder:
        result = recorder.return_to_start()
        # 可以在返回信息中带上完整的路径字符串，方便你复制
        if result["success"]:
            result["full_path_str"] = recorder.get_path_string()
        return jsonify(result)
    return jsonify({"success": False, "message": "Recorder未初始化"})

@app.route('/dog/record/status', methods=['GET'])
@require_robot
def record_status(controller):
    if recorder:
        return jsonify({
            "success": True,
            "count": len(recorder.recorded_path),
            "path_str": recorder.get_path_string()
        })
    return jsonify({"success": False})

@app.route('/dog/record/stop', methods=['POST'])
@require_robot
def stop_recording_endpoint(controller):
    """新增接口：停止录制"""
    if recorder:
        return jsonify(recorder.stop_recording())
    return jsonify({"success": False, "message": "Recorder未初始化"})

@app.route('/dog/record/delete', methods=['POST'])
@require_robot
def delete_point_endpoint(controller):
    """新增接口：静默删除"""
    if recorder:
        return jsonify(recorder.delete_last_point_data_only())
    return jsonify({"success": False, "message": "Recorder未初始化"})

@app.route('/dog/patrol/return_safely', methods=['POST'])
@require_robot
def patrol_return_safely(controller):
    """巡逻中安全返航"""
    if patrol_manager:
        return jsonify(patrol_manager.return_safely())
    return jsonify({"success": False, "message": "PatrolManager未初始化"})


@app.route('/dog/action', methods=['POST'])
@require_robot
def dog_action(controller):
    # 收到动作指令时，先停止巡逻
    if patrol_manager:
        patrol_manager.stop_patrol()

    data = request.json
    action = data.get('action', '').lower()

    if action == "greet_auto":
        print("🤖 收到自动打招呼请求...")

        was_patrolling = False
        if patrol_manager and patrol_manager.is_patrolling:
            was_patrolling = True
            print("   -> 正在巡逻，执行暂停...")
            patrol_manager.pause()
            time.sleep(0.5)  # 等待完全停稳

        try:
            # 执行打招呼流程 (共约 6-7 秒)
            print("   -> 执行 GREET 动作")
            controller.voice_command("GREET")
            # GREET 动作本身需要时间，这里等待 6 秒
            time.sleep(6)
            controller.stop_voice_command()

            # 恢复站立姿态，防止趴在地上
            controller.stand_up(seconds=1)

        except Exception as e:
            print(f"   ❌ 动作执行出错: {e}")

        # 恢复巡逻
        if was_patrolling:
            print("   -> 恢复巡逻...")
            controller.switch_to_move_mode()  # 确保切回移动模式
            time.sleep(0.5)
            patrol_manager.resume()

        return jsonify({"success": True, "message": "自动打招呼已完成"})

    if action == "stand_up":
        controller.stand_up()
    elif action == "sit":
        controller.sit()
    elif action == "nod":
        controller.voice_command("NOD")
        time.sleep(3)  # 缩短时间，避免阻塞太久
        controller.stop_voice_command()
        controller.stand_up()
    elif action == "shake":
        controller.voice_command("SHAKE")
        time.sleep(3)
        controller.stop_voice_command()
        controller.stand_up()
    elif action == "greet":
        controller.voice_command("GREET")
        time.sleep(3)
        controller.stop_voice_command()
        controller.stand_up()
    elif action == "dance":
        controller.voice_command("DANCE")
        time.sleep(5)
        controller.stop_voice_command()
        controller.stand_up()
    elif action == "turn_left_90":
        print("执行精准左转90度")
        # 优先尝试闭环控制
        if ROS_AVAILABLE and patrol_manager:
            execute_precise_turn(controller, patrol_manager, 90)  # 左转90
        else:
            # 降级方案：如果没有ROS，还是用时间控制
            controller.turn_left(seconds=2)
    elif action == "turn_right_90":
        print("执行精准右转90度")
        if ROS_AVAILABLE and patrol_manager:
            execute_precise_turn(controller, patrol_manager, -90)  # 右转-90
        else:
            controller.turn_right(seconds=2)
    elif action == "turn_180":  # 假设你想把掉头也加进去
        print("执行精准掉头180度")
        if ROS_AVAILABLE and patrol_manager:
            execute_precise_turn(controller, patrol_manager, 180)
        else:
            controller.turn_left(seconds=4)
    elif action == "stop":
        # 1. 停止巡逻逻辑 (在函数开头已调用，这里再次确认)
        if patrol_manager:
            patrol_manager.stop_patrol()
            # 打印日志确认坐标当前状态
            print(f"停止后当前Relative Pose: {patrol_manager.latest_pose}")

        # 2. 停止持续移动线程 (停止不断发送的指令)
        controller.stop_continuous_move()

        # 3. 发送零速度指令 (只控制电机停止，不切换机器狗模式)
        # 不再调用 controller.stop_voice_command()，因为它可能发送 CMD_STOP 导致模态复位
        controller.move(0, 0, 0)

        # 4. 确保处于移动模式 (防止意外跳出)
        controller.switch_to_move_mode()
    else:
        return jsonify({"success": False, "message": f"不支持的动作: {action}"})

    return jsonify({"success": True, "message": f"动作 {action} 执行成功"})


@app.route('/dog/move', methods=['POST'])
@require_robot
def move_dog(controller):
    # 手动控制移动时，停止巡逻
    if patrol_manager:
        patrol_manager.stop_patrol()

    data = request.json
    x = data.get("x", 0.0)
    y = data.get("y", 0.0)
    turn = data.get("turn", 0.0)
    duration = data.get("duration", 0)

    # 获取当前坐标的辅助函数
    def get_current_pose_data():
        if patrol_manager:
            # 返回格式: [x, y, yaw]
            return list(patrol_manager.latest_pose)
        return [0.0, 0.0, 0.0]

    # 定义标准返回结构
    response_data = {
        "success": True
    }

    if duration > 0:
        controller.start_continuous_move(forward_speed=x, side_speed=y, turn_speed=turn)
        time.sleep(duration)
        controller.stop_continuous_move()
        time.sleep(0.5)

        final_pose = get_current_pose_data()
        response_data["message"] = f"移动 {duration}秒已完成"
        response_data["final_pose"] = final_pose
    else:
        controller.start_continuous_move(forward_speed=x, side_speed=y, turn_speed=turn)
        current_pose = get_current_pose_data()
        response_data["message"] = "开始持续移动"
        response_data["start_pose"] = current_pose

    # 【新增逻辑：如果录制器有数据，顺便一起返回】
    if recorder and len(recorder.recorded_path) > 0:
        response_data["recording_status"] = {
            "count": len(recorder.recorded_path),
            "path": recorder.recorded_path,  # 返回 List 对象
            "path_str": recorder.get_path_string()  # 返回字符串供显示
        }

    return jsonify(response_data)


@app.route('/dog/turn_precise', methods=['POST'])
@require_robot
def turn_precise_endpoint(controller):
    """
    闭环精准转向接口
    接收 {"angle": 45.0}
    """
    data = request.json
    angle = data.get('angle', 0.0)

    if angle == 0:
        return jsonify({"success": True, "message": "角度为0，无需旋转"})

    print(f"收到精准转向指令: {angle}度")

    # 停止当前的任何巡逻
    if patrol_manager:
        patrol_manager.stop_patrol()

    # 调用你现有的闭环控制函数
    # 注意：execute_precise_turn 已经在你的代码里定义了，直接用
    success = execute_precise_turn(controller, patrol_manager, angle)

    if success:
        return jsonify({"success": True, "message": f"已完成转向 {angle} 度"})
    else:
        return jsonify({"success": False, "message": "转向失败或ROS未连接"}), 500


@app.route('/dog/patrol', methods=['POST'])
@require_robot
def dog_patrol(controller):
    data = request.json
    points = data.get('points', [])
    mode = data.get('mode', 'ONE_WAY') # 获取模式参数

    if not points:
        return jsonify({"success": False, "message": "未提供路径点"})

    if not ROS_AVAILABLE:
        return jsonify({"success": False, "message": "ROS环境未加载，无法巡逻"}), 500

    if patrol_manager:
        # 调用新的 start_patrol 方法
        patrol_manager.start_patrol(points, mode)
        return jsonify({"success": True, "message": f"开始巡逻 {len(points)} 个点位，模式: {mode}"})
    else:
        return jsonify({"success": False, "message": "巡逻管理器未初始化"}), 500


@app.route('/dog/shutdown', methods=['POST'])
@require_robot
def shutdown_dog(controller):
    global robot
    if patrol_manager:
        patrol_manager.stop_patrol()
        patrol_manager.running = False

    controller.stop()
    controller.stop_continuous_move()
    time.sleep(0.5)
    controller.stand_down(seconds=2)
    time.sleep(2)
    controller.close()

    with robot_lock:
        robot = None

    return jsonify({"success": True, "message": "机器狗已关闭"})


@app.route('/dog/lie_down', methods=['POST'])
@require_robot
def lie_down(controller):
    if patrol_manager:
        patrol_manager.stop_patrol()

    controller.stop()
    controller.stop_continuous_move()
    time.sleep(0.5)
    controller.stand_down(seconds=2)
    return jsonify({"success": True, "message": "机器狗已趴下"})


if __name__ == '__main__':
    # 监听 5007 端口
    print("启动全功能机器狗服务 on port 5007...")
    app.run(host='0.0.0.0', port=5007)