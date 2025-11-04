from robot_dog_controller import RobotDogController
import asyncio
import keyboard


async def main():
    # 创建控制器实例
    controller = RobotDogController(
        ip_address="192.168.1.121",
        port=9090,
        acceleration_time=1.0,
        deceleration_time=0.5,
        ping_interval=1.0,
        ping_timeout=3.0,
    )

    # 注册事件处理函数
    controller.subscribe_on_connected(lambda: print("已连接到机器人"))
    controller.subscribe_on_disconnected(lambda: print("断开连接"))
    controller.subscribe_on_error(lambda err: print(f"错误: {err['message']}"))

    # 连接机器人
    if not await controller.connect():
        print("连接失败，退出程序")
        return

    print("机器狗控制器已启动")
    print("按键说明:")
    print("  W/S: 前进/后退 (渐变加速)")
    print("  A/D: 左移/右移 (渐变加速)")
    print("  Q/E: 左转/右转 (渐变加速)")
    print("  Space: 站立/趴下切换")
    print("  R: 回零姿态")
    print("  P: 软急停")
    print("  1/2: 自动/手动模式")
    print("  Ctrl+C: 退出程序")

    # 键盘控制
    forward_speed = 0.0
    lateral_speed = 0.0
    angular_speed = 0.0

    try:
        while True:
            # 处理前进后退
            if keyboard.is_pressed("w"):
                print("press w")
                forward_speed = controller.max_linear_speed
            elif keyboard.is_pressed("s"):
                forward_speed = -controller.max_linear_speed
            else:
                print("in else")
                forward_speed = 0.0
            print(forward_speed)

            # 处理左右移动
            if keyboard.is_pressed("a"):
                lateral_speed = controller.max_linear_speed
            elif keyboard.is_pressed("d"):
                lateral_speed = -controller.max_linear_speed
            else:
                lateral_speed = 0.0

            # 处理旋转
            if keyboard.is_pressed("q"):
                angular_speed = controller.max_angular_speed
            elif keyboard.is_pressed("e"):
                angular_speed = -controller.max_angular_speed
            else:
                angular_speed = 0.0

            # 发送速度
            await controller.send_speed(forward_speed, lateral_speed, angular_speed)

            # 处理其他动作按键
            if keyboard.is_pressed("space"):
                await controller.toggle_posture()
                await asyncio.sleep(0.2)  # 防止重复触发

            if keyboard.is_pressed("r"):
                await controller.reset_pose()
                await asyncio.sleep(0.2)

            if keyboard.is_pressed("p"):
                await controller.emergency_stop()
                await asyncio.sleep(0.2)

            if keyboard.is_pressed("1"):
                await controller.switch_to_auto_mode()
                await asyncio.sleep(0.2)

            if keyboard.is_pressed("2"):
                await controller.switch_to_manual_mode()
                await asyncio.sleep(0.2)

            await asyncio.sleep(0.05)  # 控制循环频率

    except KeyboardInterrupt:
        print("\n程序已退出")
    except Exception as e:
        print(f"程序运行出错: {e}")
    finally:
        await controller.disconnect()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n程序已退出")
    except Exception as e:
        print(f"程序运行出错: {e}")
