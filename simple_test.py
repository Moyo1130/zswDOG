#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人控制器简单测试程序
快速测试基本功能

使用方法:
    python simple_test.py

作者: AI Assistant
日期: 2025-10-29
"""

import time
from totalController import Controller

# 机器人配置
ROBOT_IP = "192.168.1.120"  # 修改为你的机器人IP
ROBOT_PORT = 43893

def main():
    print("\n🤖 机器人简单测试程序")
    print(f"连接到: {ROBOT_IP}:{ROBOT_PORT}\n")
    
    # 使用上下文管理器，自动管理资源
    with Controller((ROBOT_IP, ROBOT_PORT)) as robot:
        
        print("="*50)
        print("步骤1: 完整初始化")
        print("="*50)
        robot.initialize()
        
        print("\n" + "="*50)
        print("步骤2: 基本动作测试")
        print("="*50)
        
        # 打招呼
        print("\n➤ 打招呼")
        robot.voice_command("GREET")
        time.sleep(3)
        
        # 点头
        print("\n➤ 点头")
        robot.nod()
        time.sleep(2)
        
        print("\n" + "="*50)
        print("步骤3: 移动测试")
        print("="*50)
        
        # 前进
        print("\n➤ 前进2秒")
        robot.start_continuous_move(forward_speed=0.3)
        time.sleep(2)
        robot.stop_continuous_move()
        time.sleep(1)
        
        # 转圈
        print("\n➤ 原地转圈3秒")
        robot.start_continuous_move(turn_speed=0.4)
        time.sleep(3)
        robot.stop_continuous_move()
        time.sleep(1)
        
        # 后退
        print("\n➤ 后退2秒")
        robot.start_continuous_move(forward_speed=-0.3)
        time.sleep(2)
        robot.stop_continuous_move()
        
        print("\n" + "="*50)
        print("✓ 测试完成！")
        print("="*50 + "\n")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  测试被中断")
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
