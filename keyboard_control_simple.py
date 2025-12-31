#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘控制机器狗脚本 - 标准库版本
使用标准库实现键盘控制（无需额外依赖）

使用方法:
    python keyboard_control_simple.py

控制说明:
    在提示符下输入命令并回车执行
    
命令列表:
    [初始化与状态]
    init    - 初始化机器狗
    stop    - 软急停
    stand   - 趴下/站立切换
    quit    - 退出程序
    
    [移动控制]
    w       - 前进
    s       - 后退
    a       - 左移
    d       - 右移
    j       - 左转
    l       - 右转
    k       - 停止移动
    
    [步态切换]
    g1      - 低速步态
    g2      - 中速步态
    g3      - 高速步态
    g4      - 抓地步态
    g5      - 通用步态
    g6      - 高踏步步态
    
    [模式切换]
    move    - 切换到移动模式
    stay    - 切换到原地模式
    auto    - 切换到自动模式
    manual  - 切换到手动模式
    
    help    - 显示帮助信息

作者: AI Assistant
日期: 2025-10-29
"""

import time
import sys
import threading
from totalController import Controller

# 机器人配置
ROBOT_IP = "10.65.234.12"  # 修改为你的机器人IP
ROBOT_PORT = 43893

# 移动速度配置
FORWARD_SPEED = 0.4
BACKWARD_SPEED = -0.4
SIDE_SPEED = 0.4
TURN_SPEED = 0.4


class SimpleKeyboardController:
    """简单键盘控制器类"""
    
    def __init__(self, robot_ip, robot_port):
        """初始化控制器"""
        self.robot = Controller((robot_ip, robot_port))
        self.is_initialized = False
        self.is_standing = False
        self.is_moving = False
        self.current_gait = "low"
        self.current_control_mode = "manual"  # 默认为手动模式
        self.running = True
    
    def print_help(self):
        """打印帮助信息"""
        print("\n" + "="*70)
        print("  🎮 键盘控制机器狗 - 命令行版本")
        print("="*70)
        print("\n[初始化与状态控制]")
        print("  init   - 初始化机器狗")
        print("  stop   - 软急停")
        print("  stand  - 趴下/站立切换")
        print("  quit   - 退出程序")
        print("\n[移动控制]")
        print("  w      - 前进（持续2秒）")
        print("  s      - 后退（持续2秒）")
        print("  a      - 左移（持续2秒）")
        print("  d      - 右移（持续2秒）")
        print("  j      - 左转（持续2秒）")
        print("  l      - 右转（持续2秒）")
        print("  k      - 停止移动")
        print("\n[步态切换]")
        print("  g1     - 低速步态")
        print("  g2     - 中速步态")
        print("  g3     - 高速步态")
        print("  g4     - 抓地步态")
        print("  g5     - 通用步态")
        print("  g6     - 高踏步步态")
        print("\n[模式切换]")
        print("  move   - 切换到移动模式")
        print("  stay   - 切换到原地模式")
        print("  auto   - 切换到自动模式")
        print("  manual - 切换到手动模式")
        print("\n[其他]")
        print("  help   - 显示此帮助信息")
        print("="*70)
        print("\n💡 提示: 输入命令后按回车执行\n")
    
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
            print("✓ 已急停\n")
        except Exception as e:
            print(f"❌ 急停失败: {e}\n")
    
    def toggle_stand(self):
        """趴下/站立切换"""
        if not self.is_initialized:
            print("⚠️  请先初始化机器狗 (输入: init)")
            return
        
        try:
            self.robot.stand_toggle()
            self.is_standing = not self.is_standing
            status = "站立" if self.is_standing else "趴下"
            print(f"✓ 已切换到{status}状态\n")
            time.sleep(0.1)
        except Exception as e:
            print(f"❌ 切换失败: {e}\n")
    
    def move_robot(self, direction, duration=1):
        """移动机器人"""
        if not self.is_initialized:
            print("⚠️  请先初始化机器狗 (输入: init)")
            return
        
        directions = {
            'w': ('前进', FORWARD_SPEED, 0, 0),
            's': ('后退', BACKWARD_SPEED, 0, 0),
            'a': ('左移', 0, -SIDE_SPEED, 0),
            'd': ('右移', 0, SIDE_SPEED, 0),
            'j': ('左转', 0, 0, -TURN_SPEED),
            'l': ('右转', 0, 0, TURN_SPEED),
        }
        
        if direction not in directions:
            return
        
        name, forward, side, turn = directions[direction]
        
        try:
            print(f"→ {name} ({duration}秒)...")
            self.robot.start_continuous_move(forward, side, turn)
            self.is_moving = True
            time.sleep(duration)
            self.robot.stop_continuous_move()
            self.is_moving = False
            print("  完成\n")
        except Exception as e:
            print(f"❌ 移动失败: {e}\n")
            self.robot.stop_continuous_move()
            self.is_moving = False
    
    def stop_moving(self):
        """停止移动"""
        if not self.is_initialized:
            print("⚠️  请先初始化机器狗 (输入: init)")
            return
        
        try:
            if self.is_moving:
                self.robot.stop_continuous_move()
                self.is_moving = False
                print("■ 已停止移动\n")
            else:
                print("⚠️  机器狗当前未在移动\n")
        except Exception as e:
            print(f"❌ 停止失败: {e}\n")
    
    def set_gait(self, gait_type):
        """设置步态"""
        if not self.is_initialized:
            print("⚠️  请先初始化机器狗 (输入: init)")
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
        """切换运动模式"""
        if not self.is_initialized:
            print("⚠️  请先初始化机器狗 (输入: init)")
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
    
    def switch_control_mode(self, mode):
        """切换控制模式（自动/手动）"""
        if not self.is_initialized:
            print("⚠️  请先初始化机器狗 (输入: init)")
            return
        
        try:
            if mode == "auto":
                self.robot.switch_to_auto_mode()
                self.current_control_mode = "auto"
                print("✓ 已切换到自动模式\n")
            elif mode == "manual":
                self.robot.switch_to_manual_mode()
                self.current_control_mode = "manual"
                print("✓ 已切换到手动模式\n")
            time.sleep(0.1)
        except Exception as e:
            print(f"❌ 控制模式切换失败: {e}\n")
    
    def process_command(self, command):
        """处理命令"""
        cmd = command.strip().lower()
        
        if not cmd:
            return
        
        # 初始化与状态控制
        if cmd == 'init':
            self.initialize_robot()
        elif cmd == 'stop':
            self.emergency_stop()
        elif cmd == 'stand':
            self.toggle_stand()
        elif cmd == 'quit' or cmd == 'q' or cmd == 'exit':
            print("\n👋 退出程序...")
            self.running = False
        
        # 移动控制
        elif cmd in ['w', 's', 'a', 'd', 'j', 'l']:
            self.move_robot(cmd)
        elif cmd == 'k':
            self.stop_moving()
        
        # 步态切换
        elif cmd == 'g1':
            self.set_gait('low')
        elif cmd == 'g2':
            self.set_gait('medium')
        elif cmd == 'g3':
            self.set_gait('high')
        elif cmd == 'g4':
            self.set_gait('grasp')
        elif cmd == 'g5':
            self.set_gait('general')
        elif cmd == 'g6':
            self.set_gait('high_step')
        
        # 模式切换
        elif cmd == 'move':
            self.switch_mode('move')
        elif cmd == 'stay':
            self.switch_mode('stay')
        elif cmd == 'auto':
            self.switch_control_mode('auto')
        elif cmd == 'manual':
            self.switch_control_mode('manual')
        
        # 帮助
        elif cmd == 'help' or cmd == 'h':
            self.print_help()
        
        else:
            print(f"❌ 未知命令: {cmd}")
            print("   输入 'help' 查看帮助\n")
    
    def run(self):
        """运行控制器"""
        self.print_help()
        print("✓ 控制器已启动！")
        print("💡 输入 'init' 开始初始化机器狗\n")
        
        try:
            while self.running:
                try:
                    # 读取用户输入
                    command = input("🎮 > ")
                    self.process_command(command)
                except EOFError:
                    print("\n检测到输入结束，退出...")
                    break
                except KeyboardInterrupt:
                    print("\n检测到 Ctrl+C，退出...")
                    break
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print("\n正在清理资源...")
        
        # 停止移动
        if self.is_moving:
            try:
                self.robot.stop_continuous_move()
            except:
                pass
        
        # 关闭机器人连接
        try:
            self.robot.close()
        except:
            pass
        
        print("✓ 资源已清理")
        print("👋 再见！\n")


def main():
    """主函数"""
    print("\n" + "="*70)
    print("  🎮 键盘控制机器狗程序 - 命令行版本")
    print("="*70)
    print(f"\n目标机器人: {ROBOT_IP}:{ROBOT_PORT}")
    
    # 创建并运行控制器
    try:
        controller = SimpleKeyboardController(ROBOT_IP, ROBOT_PORT)
        controller.run()
    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n程序已退出")


if __name__ == "__main__":
    main()
