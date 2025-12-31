#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器狗位姿闭环控制程序
实现让机器狗移动到指定的世界坐标 (px, py)

使用方法:
    python pose_control.py

控制说明:
    init        - 初始化机器狗
    goto x y    - 移动到指定坐标 (例如: goto 1.0 0.5)
    stop        - 停止移动/急停
    info        - 显示当前位姿
    quit        - 退出程序

注意:
    需要确保机器狗已连接且 ROS 环境已配置，能够接收 /leg_odom 话题。
"""

import time
import math
import threading
import sys
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from totalController import Controller

# 配置
ROBOT_IP = "192.168.1.120"
CMD_PORT = 43893    # 指令端口

class PoseController:
    def __init__(self, robot_ip, cmd_port):
        self.controller = Controller((robot_ip, cmd_port))
        
        # 初始化 ROS 节点和订阅者
        # 检查节点是否已初始化，避免重复初始化
        if rospy.get_node_uri() is None:
            rospy.init_node('pose_controller', anonymous=True)
        
        rospy.Subscriber("/leg_odom", PoseWithCovarianceStamped, self.pose_callback)
        self.latest_pose_data = None

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0 # Optional: target heading
        
        self.running = False
        self.control_thread = None
        self.lock = threading.Lock()
        self.is_initialized = False
        
        # PID 参数
        self.kp_linear = 1.5      # 位置比例增益
        self.kp_angular = 1.0     # 角度比例增益
        
        # 限制
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5
        self.distance_tolerance = 0.05  # 到达目标的距离容差 (米)
        
        # 状态
        self.current_pose = None
        self.is_moving_to_target = False

    def pose_callback(self, msg):
        """ROS 回调函数，处理位姿数据"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        x = position.x
        y = position.y
        
        # 四元数转 Yaw 角
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        
        # yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.latest_pose_data = (x, y, yaw)

    def initialize(self):
        """初始化机器人"""
        print("正在初始化机器人...")
        try:
            self.controller.initialize()
            self.is_initialized = True
            print("初始化完成！")
            
            # 获取初始位姿作为当前目标，防止刚启动就乱跑
            pose = self.get_pose()
            if pose:
                self.target_x, self.target_y, _ = pose
                print(f"当前位置已设为初始目标: ({self.target_x:.2f}, {self.target_y:.2f})")
                
        except Exception as e:
            print(f"初始化失败: {e}")

    def get_pose(self):
        """获取当前位姿"""
        return self.latest_pose_data

    def set_target(self, x, y):
        """设置目标位置"""
        with self.lock:
            self.target_x = float(x)
            self.target_y = float(y)
            self.is_moving_to_target = True
            print(f"设定新目标: ({self.target_x:.2f}, {self.target_y:.2f})")

    def stop_robot(self):
        """停止机器人"""
        with self.lock:
            self.is_moving_to_target = False
        self.controller.stop_continuous_move()
        print("机器人已停止")

    def _control_loop(self):
        """控制循环线程"""
        print("控制循环已启动")
        while self.running:
            if not self.is_initialized:
                time.sleep(1.0)
                continue

            # 获取当前位姿
            pose = self.get_pose()
            if pose is None:
                # print("警告: 无法获取位姿数据")
                time.sleep(0.1)
                continue
            
            self.current_pose = pose
            curr_x, curr_y, curr_yaw = pose

            # 检查是否需要移动
            with self.lock:
                moving = self.is_moving_to_target
                tgt_x = self.target_x
                tgt_y = self.target_y

            if not moving:
                time.sleep(0.1)
                continue

            # 计算全局误差
            err_x = tgt_x - curr_x
            err_y = tgt_y - curr_y
            distance = math.sqrt(err_x**2 + err_y**2)

            # 检查是否到达目标
            if distance < self.distance_tolerance:
                print(f"已到达目标! 误差: {distance:.3f}m")
                with self.lock:
                    self.is_moving_to_target = False
                self.controller.stop_continuous_move()
                continue

            # 计算全局期望速度 (P控制)
            v_gx = self.kp_linear * err_x
            v_gy = self.kp_linear * err_y

            # 速度限幅
            v_mag = math.sqrt(v_gx**2 + v_gy**2)
            if v_mag > self.max_linear_speed:
                scale = self.max_linear_speed / v_mag
                v_gx *= scale
                v_gy *= scale

            # 坐标变换: 全局坐标系 -> 机器人坐标系
            # 机器人坐标系: x前方, y左方 (通常ROS标准)
            # 旋转矩阵 R(theta) = [[cos, -sin], [sin, cos]]
            # V_global = R * V_robot
            # V_robot = R^T * V_global
            # V_rx =  cos(theta) * V_gx + sin(theta) * V_gy
            # V_ry = -sin(theta) * V_gx + cos(theta) * V_gy
            
            sin_yaw = math.sin(curr_yaw)
            cos_yaw = math.cos(curr_yaw)

            v_rx = cos_yaw * v_gx + sin_yaw * v_gy
            v_ry = -sin_yaw * v_gx + cos_yaw * v_gy

            # 角度控制: 始终朝向目标点位
            target_heading = math.atan2(err_y, err_x)
            heading_err = target_heading - curr_yaw
            
            # 归一化误差到 [-pi, pi]
            while heading_err > math.pi:
                heading_err -= 2 * math.pi
            while heading_err < -math.pi:
                heading_err += 2 * math.pi
            
            # 计算转向速度 (P控制)
            # 注意: 机器狗接口 turn_speed 正值向右转(顺时针)，而我们需要逆时针旋转来增加 Yaw
            # 因此需要取反
            turn_speed = -self.kp_angular * heading_err
            
            # 转向速度限幅
            if turn_speed > self.max_angular_speed:
                turn_speed = self.max_angular_speed
            elif turn_speed < -self.max_angular_speed:
                turn_speed = -self.max_angular_speed

            # 发送控制指令
            # 注意: totalController.py 中的 start_continuous_move 参数顺序是 (forward, side, turn)
            # 且 side_speed 正值向右? 
            # totalController.py 注释: "MOVE_Y ... 正值向右"
            # ROS标准通常是Y向左。我们需要确认坐标系定义。
            # 假设 ros_send.py 返回的是标准ROS坐标系 (X前, Y左, Z上)
            # totalController.py: MOVE_X 正值向前, MOVE_Y 正值向右
            # 如果我们的 v_ry 是基于 Y向左计算的，那么传给 controller 时需要取反?
            # 让我们再检查一下 totalController.py 的注释:
            # "MOVE_Y (0x21010131): 左右平移... 正值向右"
            # 如果 ros_send.py 的 quaternion_to_yaw 是标准转换，那么 yaw 是逆时针为正。
            # 如果我们计算出的 v_ry 是向左的速度 (ROS标准Y轴)，那么发送给机器狗时应该是负值 (因为机器狗协议Y是向右)。
            
            # 修正: 机器狗协议 side_speed 正值向右。
            # 我们的 v_ry 是沿着机器人局部坐标系Y轴的速度。
            # 如果我们遵循ROS习惯 (Y左)，那么 v_ry > 0 意味着向左。
            # 此时应该发送 side_speed = -v_ry。
            
            # 让我们验证一下 ros_send.py 的坐标系。
            # 通常 /leg_odom 是符合 ROS REP-103 的 (X前, Y左, Z上)。
            # 所以 v_ry 是向左的速度。
            # 机器狗接口: side_speed 正向右。
            # 所以 side_speed = -v_ry。
            
            self.controller.start_continuous_move(v_rx, -v_ry, turn_speed)
            
            # 打印调试信息 (每10次循环打印一次，避免刷屏)
            # print(f"Dist: {distance:.2f}, V_rob: ({v_rx:.2f}, {v_ry:.2f})")
            
            time.sleep(0.05) # 20Hz

    def start(self):
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

    def stop(self):
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        self.controller.stop_continuous_move()
        self.controller.close()

def main():
    print("="*50)
    print("机器狗位姿闭环控制程序")
    print("="*50)
    
    pose_controller = PoseController(ROBOT_IP, CMD_PORT)
    pose_controller.start()
    
    print("输入 'help' 查看指令列表")
    
    try:
        while True:
            cmd_str = input("CMD > ").strip()
            if not cmd_str:
                continue
                
            parts = cmd_str.split()
            cmd = parts[0].lower()
            
            if cmd == 'quit' or cmd == 'exit':
                break
            elif cmd == 'help':
                print("指令列表:")
                print("  init        - 初始化机器狗")
                print("  goto x y    - 移动到指定坐标 (例如: goto 1.0 0.5)")
                print("  stop        - 停止移动")
                print("  info        - 显示当前位姿")
                print("  quit        - 退出")
            elif cmd == 'init':
                pose_controller.initialize()
            elif cmd == 'stop':
                pose_controller.stop_robot()
            elif cmd == 'info':
                pose = pose_controller.get_pose()
                if pose:
                    print(f"当前位姿: X={pose[0]:.3f}, Y={pose[1]:.3f}, Yaw={pose[2]:.3f} rad")
                else:
                    print("无法获取位姿")
            elif cmd == 'goto':
                if len(parts) != 3:
                    print("参数错误。用法: goto x y")
                    continue
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    pose_controller.set_target(x, y)
                except ValueError:
                    print("坐标必须是数字")
            else:
                print("未知指令")
                
    except KeyboardInterrupt:
        print("\n程序中断")
    finally:
        print("正在关闭...")
        pose_controller.stop()
        print("再见")

if __name__ == "__main__":
    main()
