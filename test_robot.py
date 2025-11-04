#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人控制器测试程序
测试totalController.py中的各项功能

使用方法:
    python test_robot.py

作者: AI Assistant
日期: 2025-10-29
"""

import time
import sys
from totalController import Controller

# 机器人IP和端口配置
ROBOT_IP = "192.168.1.120"  # 修改为你的机器人IP地址
ROBOT_PORT = 43893

def print_section(title):
    """打印测试章节标题"""
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60)

def test_basic_connection(controller):
    """测试1: 基本连接和心跳"""
    print_section("测试1: 基本连接和心跳")
    
    try:
        print("发送心跳信号...")
        controller.heartbeat()
        time.sleep(1)
        print("✓ 心跳发送成功")
        return True
    except Exception as e:
        print(f"❌ 心跳测试失败: {e}")
        return False

def test_initialization(controller):
    """测试2: 完整初始化流程"""
    print_section("测试2: 完整初始化流程")
    
    try:
        success = controller.initialize()
        if success:
            print("✓ 初始化流程完成")
        return success
    except Exception as e:
        print(f"❌ 初始化失败: {e}")
        return False

def test_voice_commands(controller):
    """测试3: 语音指令"""
    print_section("测试3: 语音指令测试")
    
    try:
        print("\n[3.1] 测试打招呼...")
        controller.voice_command("GREET")
        time.sleep(3)
        
        print("\n[3.2] 测试低头...")
        controller.voice_command("HEAD_DOWN")
        time.sleep(1)
        
        print("\n[3.3] 测试抬头...")
        controller.voice_command("HEAD_UP")
        time.sleep(1)
        
        print("\n[3.4] 测试向左看...")
        controller.voice_command("LOOK_LEFT")
        time.sleep(1)
        
        print("\n[3.5] 测试向右看...")
        controller.voice_command("LOOK_RIGHT")
        time.sleep(1)
        
        print("✓ 语音指令测试完成")
        return True
    except Exception as e:
        print(f"❌ 语音指令测试失败: {e}")
        return False

def test_basic_movement(controller):
    """测试4: 基本移动"""
    print_section("测试4: 基本移动测试")
    
    try:
        print("\n[4.1] 前进2秒...")
        controller.start_continuous_move(forward_speed=0.3)
        time.sleep(2)
        controller.stop_continuous_move()
        time.sleep(1)
        
        print("\n[4.2] 后退2秒...")
        controller.start_continuous_move(forward_speed=-0.3)
        time.sleep(2)
        controller.stop_continuous_move()
        time.sleep(1)
        
        print("\n[4.3] 向左转2秒...")
        controller.start_continuous_move(turn_speed=-0.3)
        time.sleep(2)
        controller.stop_continuous_move()
        time.sleep(1)
        
        print("\n[4.4] 向右转2秒...")
        controller.start_continuous_move(turn_speed=0.3)
        time.sleep(2)
        controller.stop_continuous_move()
        time.sleep(1)
        
        print("✓ 基本移动测试完成")
        return True
    except Exception as e:
        print(f"❌ 基本移动测试失败: {e}")
        controller.stop_continuous_move()
        return False

def test_gait_switching(controller):
    """测试5: 步态切换"""
    print_section("测试5: 步态切换测试")
    
    try:
        gaits = ['low', 'medium', 'high']
        
        for gait in gaits:
            print(f"\n[5.{gaits.index(gait)+1}] 切换到{gait}速步态并前进2秒...")
            controller.set_gait(gait)
            time.sleep(1)
            controller.start_continuous_move(forward_speed=0.4)
            time.sleep(2)
            controller.stop_continuous_move()
            time.sleep(2)
        
        print("✓ 步态切换测试完成")
        return True
    except Exception as e:
        print(f"❌ 步态切换测试失败: {e}")
        controller.stop_continuous_move()
        return False

def test_complex_movement(controller):
    """测试6: 复杂运动组合"""
    print_section("测试6: 复杂运动组合")
    
    try:
        print("\n[6.1] 前进+右转 (组合运动)...")
        controller.start_continuous_move(forward_speed=0.3, turn_speed=0.2)
        time.sleep(3)
        controller.stop_continuous_move()
        time.sleep(1)
        
        print("\n[6.2] 后退+左转 (组合运动)...")
        controller.start_continuous_move(forward_speed=-0.3, turn_speed=-0.2)
        time.sleep(3)
        controller.stop_continuous_move()
        time.sleep(1)
        
        print("\n[6.3] 侧移测试 (向右侧移)...")
        controller.start_continuous_move(side_speed=0.4)
        time.sleep(2)
        controller.stop_continuous_move()
        time.sleep(1)
        
        print("✓ 复杂运动测试完成")
        return True
    except Exception as e:
        print(f"❌ 复杂运动测试失败: {e}")
        controller.stop_continuous_move()
        return False

def test_emergency_stop(controller):
    """测试7: 紧急停止"""
    print_section("测试7: 紧急停止测试")
    
    try:
        print("\n开始运动...")
        controller.start_continuous_move(forward_speed=0.3)
        time.sleep(1)
        
        print("触发紧急停止...")
        controller.emergency_stop()
        controller.stop_continuous_move()
        time.sleep(2)
        
        print("✓ 紧急停止测试完成")
        return True
    except Exception as e:
        print(f"❌ 紧急停止测试失败: {e}")
        return False

def test_mode_switching(controller):
    """测试8: 模式切换"""
    print_section("测试8: 模式切换测试")
    
    try:
        print("\n[8.1] 切换到原地模式...")
        controller.switch_to_stay_mode()
        time.sleep(1)
        
        print("\n[8.2] 切换到移动模式...")
        controller.switch_to_move_mode()
        time.sleep(1)
        
        print("✓ 模式切换测试完成")
        return True
    except Exception as e:
        print(f"❌ 模式切换测试失败: {e}")
        return False

def run_quick_test(controller):
    """快速测试 - 只测试基本功能"""
    print_section("🚀 快速测试模式")
    
    results = []
    
    # 测试1: 连接
    results.append(("基本连接", test_basic_connection(controller)))
    
    # 测试2: 初始化
    results.append(("完整初始化", test_initialization(controller)))
    
    # 测试3: 简单移动
    print_section("快速移动测试")
    try:
        print("前进1秒...")
        controller.start_continuous_move(forward_speed=0.3)
        time.sleep(1)
        controller.stop_continuous_move()
        results.append(("快速移动", True))
    except Exception as e:
        print(f"移动测试失败: {e}")
        results.append(("快速移动", False))
    
    return results

def run_full_test(controller):
    """完整测试 - 测试所有功能"""
    print_section("🔬 完整测试模式")
    
    results = []
    
    # 依次执行所有测试
    results.append(("1. 基本连接", test_basic_connection(controller)))
    
    if results[-1][1]:  # 如果连接成功才继续
        results.append(("2. 完整初始化", test_initialization(controller)))
        results.append(("3. 语音指令", test_voice_commands(controller)))
        results.append(("4. 基本移动", test_basic_movement(controller)))
        results.append(("5. 步态切换", test_gait_switching(controller)))
        results.append(("6. 复杂运动", test_complex_movement(controller)))
        results.append(("7. 紧急停止", test_emergency_stop(controller)))
        results.append(("8. 模式切换", test_mode_switching(controller)))
    
    return results

def print_test_results(results):
    """打印测试结果汇总"""
    print_section("📊 测试结果汇总")
    
    total = len(results)
    passed = sum(1 for _, result in results if result)
    
    print(f"\n总测试数: {total}")
    print(f"通过: {passed}")
    print(f"失败: {total - passed}")
    print(f"成功率: {passed/total*100:.1f}%\n")
    
    print("详细结果:")
    for test_name, result in results:
        status = "✓ 通过" if result else "✗ 失败"
        print(f"  {status} - {test_name}")
    
    print("\n" + "="*60)

def interactive_menu(controller):
    """交互式菜单"""
    while True:
        print_section("🎮 交互式控制菜单")
        print("1. 初始化机器人")
        print("2. 前进")
        print("3. 后退")
        print("4. 左转")
        print("5. 右转")
        print("6. 停止")
        print("7. 打招呼")
        print("8. 点头")
        print("9. 摇头")
        print("0. 退出")
        print("-" * 60)
        
        try:
            choice = input("请选择操作 (0-9): ").strip()
            
            if choice == '0':
                print("退出交互模式")
                break
            elif choice == '1':
                controller.initialize()
            elif choice == '2':
                controller.start_continuous_move(forward_speed=0.3)
                time.sleep(2)
                controller.stop_continuous_move()
            elif choice == '3':
                controller.start_continuous_move(forward_speed=-0.3)
                time.sleep(2)
                controller.stop_continuous_move()
            elif choice == '4':
                controller.start_continuous_move(turn_speed=-0.3)
                time.sleep(2)
                controller.stop_continuous_move()
            elif choice == '5':
                controller.start_continuous_move(turn_speed=0.3)
                time.sleep(2)
                controller.stop_continuous_move()
            elif choice == '6':
                controller.stop_continuous_move()
            elif choice == '7':
                controller.voice_command("GREET")
                time.sleep(2)
            elif choice == '8':
                controller.nod()
            elif choice == '9':
                controller.shake()
            else:
                print("无效选择，请重新输入")
                
        except KeyboardInterrupt:
            print("\n\n检测到Ctrl+C，退出...")
            break
        except Exception as e:
            print(f"❌ 操作失败: {e}")

def main():
    """主函数"""
    print("\n" + "="*60)
    print("  🤖 机器人控制器测试程序")
    print("="*60)
    print(f"\n目标机器人: {ROBOT_IP}:{ROBOT_PORT}")
    
    # 创建控制器 (使用上下文管理器自动管理资源)
    with Controller((ROBOT_IP, ROBOT_PORT)) as controller:
        
        # 显示测试模式选择
        print("\n请选择测试模式:")
        print("1. 快速测试 (基本功能)")
        print("2. 完整测试 (所有功能)")
        print("3. 交互模式 (手动控制)")
        print("4. 仅初始化")
        
        try:
            choice = input("\n请输入选项 (1-4): ").strip()
            
            if choice == '1':
                results = run_quick_test(controller)
                print_test_results(results)
                
            elif choice == '2':
                results = run_full_test(controller)
                print_test_results(results)
                
            elif choice == '3':
                # 先初始化
                print("\n执行初始化...")
                controller.initialize()
                # 进入交互模式
                interactive_menu(controller)
                
            elif choice == '4':
                test_initialization(controller)
                print("\n✓ 初始化完成，机器人已准备就绪")
                
            else:
                print("无效选择，程序退出")
                
        except KeyboardInterrupt:
            print("\n\n检测到Ctrl+C，正在安全退出...")
        except Exception as e:
            print(f"\n❌ 程序异常: {e}")
    
    print("\n" + "="*60)
    print("  程序已安全退出，所有资源已释放")
    print("="*60 + "\n")

if __name__ == "__main__":
    main()
