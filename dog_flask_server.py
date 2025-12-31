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
        self.is_aligning = False
        self.running = True

        # 卡点保护
        self.target_start_time = 0

        self.patrol_mode = "ONE_WAY"
        self.original_path = []
        self.current_path_points = []

        # === PID 参数 ===
        self.kp_linear = 0.6
        self.kp_angular = 0.6
        self.max_linear_speed = 1.0
        self.max_angular_speed = 0.6

        # 容差配置
        self.dist_tolerance = 0.05
        self.yaw_tolerance = 0.10
        self.align_start_time = 0

        if ROS_AVAILABLE:
            if rospy.get_node_uri() is None:
                rospy.init_node('dog_flask_merged_node', anonymous=True, disable_signals=True)
            rospy.Subscriber("/leg_odom", PoseWithCovarianceStamped, self.pose_callback)
            threading.Thread(target=self.control_loop, daemon=True).start()

    def reset_origin(self):
        if self.raw_pose != (0.0, 0.0, 0.0):
            self.origin_pose = self.raw_pose
            print(f"坐标系已重置，新原点 (Raw): {self.origin_pose}")
        else:
            print("⚠️ 警告: 尚未收到odom数据，无法重置原点")

    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.raw_pose = (pos.x, pos.y, yaw)
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
            self.is_aligning = False

            if len(points) > 1:
                closest_idx = self._find_closest_point_index(points)
                if closest_idx == len(points) - 1 and self.patrol_mode == "LOOP":
                    closest_idx = 0
                start_points = points[closest_idx:]
                for p in start_points:
                    self._append_to_queue(p)
            else:
                for p in points:
                    self._append_to_queue(p)

            self.is_patrolling = True
            print(f"🚀 巡逻开始 | 模式: {mode} | 起点索引: {0 if len(points) == 1 else '智能计算'}")

    def _append_to_queue(self, p):
        if len(p) == 2:
            self.waypoints_queue.append((p[0], p[1], None))
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
        self.controller.start_continuous_move(0, 0, 0)
        time.sleep(0.1)

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
        rate = 10
        dt = 1.0 / rate

        while self.running:
            try:
                if not self.is_patrolling or not ROS_AVAILABLE:
                    time.sleep(0.2)
                    continue

                # === 1. 获取新目标与【预转向逻辑】 ===
                if self.current_target is None:
                    self.is_aligning = False
                    if len(self.waypoints_queue) > 0:
                        self.current_target = self.waypoints_queue.popleft()
                        self.target_start_time = time.time()

                        print(f"📍 前往新目标: {self.current_target}")
                        print(f"   当前位置: {self.latest_pose}")

                        # ------------【新增核心逻辑：先转再走】------------
                        # 计算需要转多少度才能面朝目标
                        turn_deg = self._calculate_required_turn_degrees(self.latest_pose, self.current_target)
                        print(f"📐 计算所需预转向角度: {turn_deg:.2f}°")

                        # 硬性判定：如果角度差异超过 40 度，我们认为是“大弯”
                        # 40度包含了你提到的 70度、80度、90度、135度的情况
                        if abs(turn_deg) > 40:
                            print(f"⚠️ 角度过大 ({turn_deg:.2f}°)，执行【原地停车转向】以防摆头...")

                            # 1. 先完全停车
                            self.controller.start_continuous_move(0, 0, 0)
                            time.sleep(0.2)

                            # 2. 调用精准转向 (这是一个阻塞操作，会一直等到转完)
                            # 传入 turn_deg，正数左转，负数右转，符合 execute_precise_turn 逻辑
                            success = execute_precise_turn(self.controller, self, turn_deg, timeout=6.0)

                            if success:
                                print(f"✅ 预转向完成，当前角度: {self.latest_pose[2]:.2f}，开始直线前往目标")
                            else:
                                print("❌ 预转向超时，尝试强制进入PID逻辑")

                            # 更新一下开始时间，防止因为转向消耗时间导致误判超时
                            self.target_start_time = time.time()

                            # ----------------------------------------------------

                    else:
                        if self.patrol_mode == "LOOP":
                            print("🔄 循环模式：本圈结束")

                            # === 核心修复开始 ===
                            # 1. 强制停车，消除运动惯性
                            self.controller.start_continuous_move(0, 0, 0)
                            time.sleep(0.5)

                            # 2. 重置原点！
                            # 这会将机器狗当前的物理位置，强制设为新的 (0,0,0) 坐标系的中心
                            # 这样，原本的第一点 [2, 0, 0] 就变成了“相对于当前位置向前2米”
                            # 从而消除了上一圈积累的里程计误差。
                            self.reset_origin()

                            # 3. 重新装填路径
                            print(f"📍 坐标系已重置，开始下一圈 (偏移已清零)")
                            for p in self.original_path:
                                self._append_to_queue(p)
                            continue
                        else:
                            print("✅ 巡逻结束")
                            self.stop_patrol()
                            continue

                if self.latest_pose is None:
                    time.sleep(dt)
                    continue

                # 2. 超时跳过检测
                if time.time() - self.target_start_time > 180.0:
                    print(f"⚠️ 卡点超时(3min)，强制跳过当前点: {self.current_target}")
                    self.current_target = None
                    continue

                # 3. 计算全局误差
                curr_x, curr_y, curr_yaw = self.latest_pose
                tgt_x, tgt_y, tgt_yaw = self.current_target

                err_x = tgt_x - curr_x
                err_y = tgt_y - curr_y
                dist = math.sqrt(err_x ** 2 + err_y ** 2)

                # 4. 判断对齐模式
                if not self.is_aligning:
                    if dist < self.dist_tolerance:
                        self.is_aligning = True
                        print(f"🎯 到达位置(误差{dist:.2f}m)，开始最后对齐...")
                else:
                    if time.time() - self.align_start_time > 6.0:
                        print("⚠️ 对齐超时(5s)，强制跳至下一目标")
                        self.current_target = None  # 跳过当前点，去下一个
                        continue
                    if dist > (self.dist_tolerance + 0.15):
                        self.is_aligning = False
                        print("⚠️ 偏离目标，重新移动...")

                # 5. 计算PID控制量
                final_v_x = 0.0
                final_v_y = 0.0
                final_w_z = 0.0

                if not self.is_aligning:
                    # === 阶段A: 移动趋近 ===
                    rel_x = err_x * math.cos(curr_yaw) + err_y * math.sin(curr_yaw)
                    rel_y = (-err_x * math.sin(curr_yaw) + err_y * math.cos(curr_yaw))

                    target_heading_local = math.atan2(rel_y, rel_x)

                    # 这里的 AIM_THRESHOLD 可以设小一点，因为我们已经做过预对齐了
                    # 此时主要是直线微调
                    AIM_THRESHOLD = 0.20  # 约11度

                    if abs(target_heading_local) > AIM_THRESHOLD:
                        # 依然保留这个分支作为保险，万一预对齐没对准
                        v_cmd_x = 0.0
                        v_cmd_y = 0.0
                        raw_w = self.kp_angular * target_heading_local

                        if abs(raw_w) < 0.05: raw_w = 0
                        w_cmd_z = self._apply_deadzone(raw_w, deadzone=0.2, max_val=0.5)
                    else:
                        # 直线前进，此时转弯分量应该很小
                        w_cmd_z = self.kp_angular * target_heading_local
                        v_cmd_x = self.kp_linear * rel_x
                        v_cmd_y = 0.0  # 尽量不要用横移，容易打滑

                        v_cmd_x = self._apply_deadzone(v_cmd_x, deadzone=0.15, max_val=0.8)
                        w_cmd_z = self._apply_deadzone(w_cmd_z, deadzone=0.15, max_val=0.4)

                    v_cmd_x = max(min(v_cmd_x, 1.0), -1.0)
                    w_cmd_z = max(min(w_cmd_z, 1.0), -1.0)

                    final_v_x = self._apply_deadzone(v_cmd_x, deadzone=0.15, max_val=0.8)
                    final_v_y = 0.0  # 强制关闭横移，防止斜跑
                    final_w_z = self._apply_deadzone(w_cmd_z, deadzone=0.15, max_val=0.8)

                    final_w_z = -final_w_z  # 方向修正

                else:
                    # === 阶段B: 到达后最终朝向对齐 ===
                    if tgt_yaw is None:
                        self.current_target = None
                        continue

                    yaw_err = tgt_yaw - curr_yaw
                    while yaw_err > math.pi: yaw_err -= 2 * math.pi
                    while yaw_err < -math.pi: yaw_err += 2 * math.pi

                    if abs(yaw_err) < self.yaw_tolerance:
                        print("✅ 精确对齐完成，前往下一目标")
                        self.current_target = None
                        continue

                    # --- 修改开始：增强对齐时的扭矩 ---
                    raw_w = self.kp_angular * yaw_err

                    # 强制最小启动速度：如果误差存在，至少给 0.35 的速度
                    min_align_speed = 0.35  # 如果地面摩擦大，改为 0.4

                    if abs(raw_w) < min_align_speed:
                        # 保持符号，但幅值强制提升到 min_align_speed
                        raw_w = math.copysign(min_align_speed, raw_w)

                    w_cmd_z = max(min(raw_w, 1.0), -1.0)  # 限幅 1.0

                    # 这里 apply_deadzone 的 deadzone 参数其实失效了，因为上面已经保底了
                    # 但为了安全 max_val 依然生效
                    final_w_z = self._apply_deadzone(w_cmd_z, deadzone=0.1, max_val=0.8)
                    final_w_z = -final_w_z

                    final_v_x = 0.0
                    final_v_y = 0.0

                self.controller.start_continuous_move(final_v_x, final_v_y, final_w_z)
                time.sleep(dt)

            except Exception as e:
                import traceback
                traceback.print_exc()
                print(f"❌ 巡逻线程异常: {e}")
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
        self.recorded_path = []  # 存储元组 list [(x, y, yaw), ...]
        self.min_dist_threshold = 0.2  # 最小记录间距，防止精度抖动导致重复点

    def clear(self):
        self.recorded_path = []
        print("路径录制已清空")


    def record_current_point(self):
        """记录当前点位，带去重逻辑"""
        curr_pose = self.patrol_ctrl.latest_pose  # (x, y, yaw)

        # 简单校验数据有效性
        if curr_pose == (0.0, 0.0, 0.0):
            return {"success": False, "message": "未能获取有效定位数据(0,0,0)"}

        # 去重逻辑：如果和上一个点距离太近，视为同一个点，更新即可或者是忽略
        if len(self.recorded_path) > 0:
            last_pose = self.recorded_path[-1]
            dist = math.sqrt((curr_pose[0] - last_pose[0]) ** 2 + (curr_pose[1] - last_pose[1]) ** 2)
            if dist < self.min_dist_threshold:
                # 距离太近，更新最后一个点为当前更精确的点，或者直接忽略
                self.recorded_path[-1] = curr_pose
                return {"success": True, "message": f"点位更新(距离过近): {curr_pose}", "point": curr_pose,
                        "count": len(self.recorded_path)}

        self.recorded_path.append(curr_pose)
        return {"success": True, "message": f"点位已记录: {curr_pose}", "point": curr_pose,
                "count": len(self.recorded_path)}

    def undo_last_point(self):
        """
        单次原路返回：
        1. 删掉当前所在的这个“错误”点（栈顶）
        2. 导航回上一个点（新栈顶）
        """
        if len(self.recorded_path) == 0:
            return {"success": False, "message": "没有可撤销的点位"}

        # 1. 删除当前点
        removed = self.recorded_path.pop()
        print(f"撤销点位: {removed}")

        if len(self.recorded_path) == 0:
            # === 修改开始 ===
            # 列表空了，说明刚才撤销的是唯一的起点。
            # 为了符合直觉，我们让狗回到原点 (0,0,0) 并保持朝向为 0
            print("所有点位已撤销，正在返回绝对原点 (0,0,0)...")
            self.patrol_ctrl.start_patrol([(0.0, 0.0, 0.0)], mode="ONE_WAY")
            return {"success": True, "message": "已撤销起点，正在返回初始原点", "remaining_count": 0}
            # === 修改结束 ===

        # 2. 获取上一个点 (这是我们要回退去的目标)
        target_pose = self.recorded_path[-1]

        # 3. 调用巡逻控制器去往该点
        print(f"正在回退至上一个点: {target_pose}")
        self.patrol_ctrl.start_patrol([target_pose], mode="ONE_WAY")

        return {"success": True, "message": f"已撤销并正在返回上一点: {target_pose}",
                "remaining_count": len(self.recorded_path)}

    def return_to_start(self):
        """
        最终原路返回：
        将记录的路径反转，然后执行巡逻
        """
        if len(self.recorded_path) < 2:
            return {"success": False, "message": "路径点过少，无需执行原路返回"}

        # 深度复制并反转
        # 【需求2】原路返回时点位不变，朝向是否要反转？
        # 用户需求：“朝向必须和最初的点位一致” -> 意味着我们要去那个点，并且拥有那个点的朝向。
        # 所以直接传递 (x, y, yaw) 即可，不用反转 Yaw。
        # 比如：我在 A 点是朝北的。去 B 点。原路返回回到 A 点，我希望狗最后是朝北停在 A 点。
        # 那就把 A 点的 (x, y, yaw_north) 发给控制器即可。

        reverse_path = list(reversed(self.recorded_path))

        print("开始原路返回，路径:", reverse_path)
        # 调用时，这些点包含 yaw，AutoPatrolController 会自动执行“阶段B”对齐朝向
        self.patrol_ctrl.start_patrol(reverse_path, mode="ONE_WAY")

        return {"success": True, "message": "开始原路返回", "points": len(reverse_path)}

    def get_path_string(self):
        """获取当前记录的所有点位，方便复制到知识库"""
        # 格式化为 JSON 样式的字符串
        path_str = "[" + ", ".join([f"({p[0]:.2f}, {p[1]:.2f})" for p in self.recorded_path]) + "]"
        return path_str

    # === 静默删除点位 ===
    def delete_last_point_data_only(self):
        """
        【需求2实现】只删除数据，机器狗不动
        """
        if len(self.recorded_path) == 0:
            return {"success": False, "message": "列表为空，无法删除"}

        removed = self.recorded_path.pop()
        print(f"已静默删除点位: {removed}")

        return {"success": True, "message": "点位已删除，机器狗保持静止", "count": len(self.recorded_path)}

    # === 停止录制（原地完成） ===
    def stop_recording(self):
        """
        【需求1实现】结束录制，保存数据，不动
        """
        if len(self.recorded_path) == 0:
            return {"success": False, "message": "没有录制数据"}

        path_str = self.get_path_string()
        print(f"录制完成，路径: {path_str}")
        return {"success": True, "message": "录制已完成并停止", "full_path_str": path_str}


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
def execute_precise_turn(controller, patrol_manager, target_angle_degrees, timeout=5.0):
    """
    闭环精准转向函数
    :param controller: 运动控制器
    :param patrol_manager: 拥有odom数据的管理器
    :param target_angle_degrees: 目标相对角度（正数为左转/逆时针，负数为右转/顺时针）
    :param timeout: 超时时间，防止死循环
    """
    if not patrol_manager or not ROS_AVAILABLE:
        print("⚠️ 无法获取 Odom 数据，无法执行精准转向")
        return False

    # 1. 获取当前角度 (弧度)
    # 注意：raw_pose 是 (x, y, yaw)
    _, _, start_yaw = patrol_manager.raw_pose

    # 2. 计算目标角度 (弧度)
    radian_delta = math.radians(target_angle_degrees)
    target_yaw = normalize_angle(start_yaw + radian_delta)

    print(f"🔄 开始精准转向: 目标增量 {target_angle_degrees}° | 起始Yaw {start_yaw:.2f} -> 目标Yaw {target_yaw:.2f}")

    # P控制器参数
    Kp = 2.0  # 比例系数，需要根据实际调整
    max_speed = 1.0  # 最大转向速度
    min_speed = 0.3  # 最小启动速度（克服静摩擦）
    tolerance = 0.05  # 容差弧度 (约 2.8度)

    start_time = time.time()

    try:
        while True:
            # 超时保护
            if time.time() - start_time > timeout:
                print("❌ 转向超时")
                break

            # 获取实时角度
            _, _, current_yaw = patrol_manager.raw_pose

            # 计算误差 (最短路径)
            error = normalize_angle(target_yaw - current_yaw)

            # 到达目标
            if abs(error) < tolerance:
                print(f"✅ 精准转向完成，最终误差: {math.degrees(error):.2f}°")
                break

            # 计算速度 (P控制)
            turn_speed = Kp * error

            # 限幅
            turn_speed = max(min(turn_speed, max_speed), -max_speed)

            # 最小速度补偿（防止接近目标时因为速度太小转不动）
            if abs(turn_speed) < min_speed:
                turn_speed = math.copysign(min_speed, turn_speed)

            # 注意：totalController.py 中 turn_speed 正值通常代表向左转（逆时针）
            # 如果发现方向反了，请将下面的 turn_speed 改为 -turn_speed
            # 根据你之前的代码逻辑：Kp * error error为正表示需要逆时针转
            # totalController 的 start_continuous_move(x, y, turn)
            # 我们直接发送指令
            controller.start_continuous_move(0, 0, -turn_speed)

            time.sleep(0.05)  # 20Hz 控制频率

    finally:
        # 无论如何，最后停止
        controller.stop_continuous_move()
        # 更新状态，防止 patrol_manager 的状态混乱
        # if patrol_manager:
        #     patrol_manager.reset_origin()  # 可选：更新一下相对坐标系
        pass

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