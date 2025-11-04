#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘控制机器狗脚本
使用键盘实时控制机器狗的各种动作

依赖安装:
    pip install keyboard

使用方法:
    python keyboard_control.py

注意: 需要管理员权限运行（Windows）或使用sudo（Linux）

控制按键说明:
    [初始化与状态]
    I - 初始化机器狗
    Q - 退出程序
    E - 软急停
    Space - 趴下/站立切换
    
    [移动控制]
    W - 前进
    S - 后退
    A - 左移
    D - 右移
    J - 左转
    L - 右转
    K - 停止移动
    
    [步态切换]
    1 - 低速步态
    2 - 中速步态
    3 - 高速步态
    4 - 抓地步态
    5 - 通用步态
    6 - 高踏步步态
    
    [模式切换]
    M - 切换到移动模式
    N - 切换到原地模式

作者: AI Assistant
日期: 2025-10-29
"""

import keyboard
import time
import sys
from totalController import Controller

# 机器人配置
ROBOT_IP = "192.168.1.120"  # 修改为你的机器人IP
ROBOT_PORT = 43893

# 移动速度配置
FORWARD_SPEED = 0.4   # 前进速度
BACKWARD_SPEED = -0.4  # 后退速度
SIDE_SPEED = 0.4      # 侧移速度
TURN_SPEED = 0.4      # 转向速度


class KeyboardController:
    """键盘控制器类"""
    
    def __init__(self, robot_ip, robot_port):
        """初始化键盘控制器"""
        self.robot = Controller((robot_ip, robot_port))
        self.is_initialized = False
        self.is_standing = False
        self.is_moving = False
        self.current_gait = "low"
        self.running = True
        
        # 移动状态
        self.move_forward = False
        self.move_backward = False
        self.move_left = False
        self.move_right = False
        self.turn_left = False
        self.turn_right = False
    
    def print_help(self):
        """打印帮助信息"""
        print("\n" + "="*70)
        print("🎮 键盘控制机器狗")
        print("="*70)
        print("\n[初始化与状态控制]")
        print("  I      - 初始化机器狗")
        print("  Q      - 退出程序")
        print("  E      - 软急停")
        print("  Space  - 趴下/站立切换")
        print("\n[移动控制]")
        print("  W      - 前进")
        print("  S      - 后退")
        print("  A      - 左移")
        print("  D      - 右移")
        print("  J      - 左转")
        print("  L      - 右转")
        print("  K      - 停止移动")
        print("\n[步态切换]")
        print("  1      - 低速步态")
        print("  2      - 中速步态")
        print("  3      - 高速步态")
        print("  4      - 抓地步态")
        print("  5      - 通用步态")
        print("  6      - 高踏步步态")
        print("\n[模式切换]")
        print("  M      - 切换到移动模式")
        print("  N      - 切换到原地模式")
        print("="*70)
        print("\n💡 提示: 程序需要管理员权限才能捕获键盘输入")
        print("         按下对应按键执行操作...\n")
    
    def initialize_robot(self):
        """初始化机器狗"""
        if self.is_initialized:
            print("⚠️  机器狗已经初始化")
            return
        
        print("\n🤖 开始初始化机器狗...")
        try:
            if self.robot.initialize():
                self.is_initialized = True
                self.is_standing = True
                print("✓ 初始化成功！机器狗已准备就绪\n")
            else:
                print("❌ 初始化失败\n")
        except Exception as e:
            print(f"❌ 初始化异常: {e}\n")
    
    def emergency_stop(self):
        """软急停"""
        print("\n🛑 执行软急停...")
        try:
            self.robot.emergency_stop()
            self.robot.stop_continuous_move()
            self.is_moving = False
            self._reset_move_flags()
            print("✓ 已急停\n")
        except Exception as e:
            print(f"❌ 急停失败: {e}\n")
    
    def toggle_stand(self):
        """趴下/站立切换"""
        if not self.is_initialized:
            print("⚠️  请先初始化机器狗 (按 I 键)")
            return
        
        try:
            self.robot.stand_toggle()
            self.is_standing = not self.is_standing
            status = "站立" if self.is_standing else "趴下"
            print(f"✓ 已切换到{status}状态\n")
            time.sleep(0.1)
        except Exception as e:
            print(f"❌ 切换失败: {e}\n")
    
    def set_gait(self, gait_type):
        """设置步态"""
        if not self.is_initialized:
            print("⚠️  请先初始化机器狗 (按 I 键)")
            return
        
        gait_names = {
            'low': '低速步态',
            'medium': '中速步态',
            'high': '高速步态',
            'grasp': '抓地步态',
            'general': '通用步态',
            'high_step': '高踏步步态'
        }
        
        try:
            self.robot.set_gait(gait_type)
            self.current_gait = gait_type
            print(f"✓ 已切换到{gait_names[gait_type]}\n")
            time.sleep(0.1)
        except Exception as e:
            print(f"❌ 步态切换失败: {e}\n")
    
    def switch_mode(self, mode):
        """切换模式"""
        if not self.is_initialized:
            print("⚠️  请先初始化机器狗 (按 I 键)")
            return
        
        try:
            if mode == "move":
                self.robot.switch_to_move_mode()
                print("✓ 已切换到移动模式\n")
            elif mode == "stay":
                self.robot.switch_to_stay_mode()
                print("✓ 已切换到原地模式\n")
            time.sleep(0.1)
        except Exception as e:
            print(f"❌ 模式切换失败: {e}\n")
    
    def _reset_move_flags(self):
        """重置所有移动标志"""
        self.move_forward = False
        self.move_backward = False
        self.move_left = False
        self.move_right = False
        self.turn_left = False
        self.turn_right = False
    
    def update_movement(self):
        """更新移动状态"""
        if not self.is_initialized:
            return
        
        # 计算合成速度
        forward_speed = 0.0
        side_speed = 0.0
        turn_speed = 0.0
        
        if self.move_forward:
            forward_speed += FORWARD_SPEED
        if self.move_backward:
            forward_speed += BACKWARD_SPEED
        if self.move_left:
            side_speed -= SIDE_SPEED
        if self.move_right:
            side_speed += SIDE_SPEED
        if self.turn_left:
            turn_speed -= TURN_SPEED
        if self.turn_right:
            turn_speed += TURN_SPEED
        
        # 判断是否在移动
        is_moving_now = (forward_speed != 0 or side_speed != 0 or turn_speed != 0)
        
        if is_moving_now:
            if not self.is_moving:
                # 从静止到移动
                self.robot.start_continuous_move(forward_speed, side_speed, turn_speed)
                self.is_moving = True
            else:
                # 更新移动参数
                self.robot.start_continuous_move(forward_speed, side_speed, turn_speed)
        else:
            if self.is_moving:
                # 从移动到静止
                self.robot.stop_continuous_move()
                self.is_moving = False
    
    def on_key_press(self, event):
        """按键按下事件"""
        key = event.name.lower()
        
        # 初始化与状态控制
        if key == 'i':
            self.initialize_robot()
        elif key == 'q':
            print("\n👋 退出程序...")
            self.running = False
        elif key == 'e':
            self.emergency_stop()
        elif key == 'space':
            self.toggle_stand()
        
        # 移动控制（按下开始移动）
        elif key == 'w':
            if not self.move_forward:
                self.move_forward = True
                print("▲ 前进")
                self.update_movement()
        elif key == 's':
            if not self.move_backward:
                self.move_backward = True
                print("▼ 后退")
                self.update_movement()
        elif key == 'a':
            if not self.move_left:
                self.move_left = True
                print("◄ 左移")
                self.update_movement()
        elif key == 'd':
            if not self.move_right:
                self.move_right = True
                print("► 右移")
                self.update_movement()
        elif key == 'j':
            if not self.turn_left:
                self.turn_left = True
                print("↶ 左转")
                self.update_movement()
        elif key == 'l':
            if not self.turn_right:
                self.turn_right = True
                print("↷ 右转")
                self.update_movement()
        elif key == 'k':
            if self.is_moving:
                self._reset_move_flags()
                self.update_movement()
                print("■ 停止")
        
        # 步态切换
        elif key == '1':
            self.set_gait('low')
        elif key == '2':
            self.set_gait('medium')
        elif key == '3':
            self.set_gait('high')
        elif key == '4':
            self.set_gait('grasp')
        elif key == '5':
            self.set_gait('general')
        elif key == '6':
            self.set_gait('high_step')
        
        # 模式切换
        elif key == 'm':
            self.switch_mode('move')
        elif key == 'n':
            self.switch_mode('stay')
    
    def on_key_release(self, event):
        """按键释放事件"""
        key = event.name.lower()
        
        # 移动控制（释放停止移动）
        if key == 'w':
            if self.move_forward:
                self.move_forward = False
                print("  停止前进")
                self.update_movement()
        elif key == 's':
            if self.move_backward:
                self.move_backward = False
                print("  停止后退")
                self.update_movement()
        elif key == 'a':
            if self.move_left:
                self.move_left = False
                print("  停止左移")
                self.update_movement()
        elif key == 'd':
            if self.move_right:
                self.move_right = False
                print("  停止右移")
                self.update_movement()
        elif key == 'j':
            if self.turn_left:
                self.turn_left = False
                print("  停止左转")
                self.update_movement()
        elif key == 'l':
            if self.turn_right:
                self.turn_right = False
                print("  停止右转")
                self.update_movement()
    
    def run(self):
        """运行键盘控制"""
        self.print_help()
        
        # 注册按键事件
        keyboard.on_press(self.on_key_press)
        keyboard.on_release(self.on_key_release)
        
        print("✓ 键盘控制已启动！")
        print("💡 按 I 键开始初始化机器狗\n")
        
        try:
            # 保持程序运行
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\n检测到 Ctrl+C，正在退出...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print("\n正在清理资源...")
        
        # 停止移动
        if self.is_moving:
            self.robot.stop_continuous_move()
        
        # 关闭机器人连接
        self.robot.close()
        
        # 取消键盘监听
        keyboard.unhook_all()
        
        print("✓ 资源已清理")
        print("👋 再见！\n")


def check_admin():
    """检查是否有管理员权限"""
    try:
        import ctypes
        return ctypes.windll.shell32.IsUserAnAdmin()
    except:
        return True  # 非Windows系统假设有权限


def main():
    """主函数"""
    print("\n" + "="*70)
    print("  🎮 键盘控制机器狗程序")
    print("="*70)
    print(f"\n目标机器人: {ROBOT_IP}:{ROBOT_PORT}")
    
    # 检查管理员权限
    if not check_admin():
        print("\n⚠️  警告: 程序需要管理员权限才能捕获键盘输入！")
        print("请以管理员身份运行此程序：")
        print("  1. 右键点击 PowerShell")
        print("  2. 选择 '以管理员身份运行'")
        print("  3. 再次运行此脚本\n")
        input("按回车键退出...")
        return
    
    # 检查keyboard库
    try:
        import keyboard
    except ImportError:
        print("\n❌ 错误: 未安装 keyboard 库")
        print("请运行以下命令安装：")
        print("  pip install keyboard\n")
        input("按回车键退出...")
        return
    
    # 创建并运行控制器
    try:
        controller = KeyboardController(ROBOT_IP, ROBOT_PORT)
        controller.run()
    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n程序已退出")


if __name__ == "__main__":
    main()
