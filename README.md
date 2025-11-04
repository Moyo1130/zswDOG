# 机器狗控制器 (RobotDogController)

## 简介

`RobotDogController` 是一个用于控制四足机器人（机器狗）的Python接口库，通过WebSocket协议与机器人通信，提供简单易用的编程接口，支持以下功能：

- 速度控制：前进、后退、左右移动、旋转
- 姿态调整：站立/趴下切换、回零姿态
- 模式切换：自动/手动模式
- 急停功能
- 心跳机制：保持连接稳定
- 事件订阅：响应连接、断开和错误事件

## 安装

确保Python版本在3.7及以上，并安装必要的依赖：

```bash
pip install websockets keyboard
```

## 基本使用

### 不同Python版本使用

目前支持Python3.9及以上版本，其他版本未测试。
请将不同版本的pyd文件放在exmaple.py目录下使用。

 - robot_dog_controller.cp39-win_amd64.pyd
 - robot_dog_controller.cp310-win_amd64.pyd
 - robot_dog_controller.cp311-win_amd64.pyd
 - robot_dog_controller.cp312-win_amd64.pyd

### 连接机器人

```python
from robot_dog_controller import RobotDogController
import asyncio

async def main():
    # 创建控制器实例
    controller = RobotDogController(
        ip_address="192.168.1.100",  # 机器人IP地址
        port=9090,                   # WebSocket端口
        max_linear_speed=0.1,        # 最大线速度
        max_angular_speed=0.2,       # 最大角速度
        acceleration_time=1.0,       # 加速时间(秒)
        deceleration_time=0.5,       # 减速时间(秒)
        ping_interval=30,            # 心跳间隔(秒)
        ping_timeout=5               # 心跳超时时间(秒)
    )

    # 注册事件回调
    controller.subscribe_on_connected(lambda: print("已连接到机器人"))
    controller.subscribe_on_disconnected(lambda: print("断开连接"))
    controller.subscribe_on_error(lambda err: print(f"错误: {err['message']}"))

    # 连接到机器人
    if await controller.connect():
        print("机器人连接成功")

        # 执行控制命令
        await controller.toggle_posture()  # 切换站立/趴下
        await asyncio.sleep(2)

        # 发送速度指令 (前进)
        await controller.send_speed(0.1, 0, 0)
        await asyncio.sleep(3)

        # 停止
        await controller.send_speed(0, 0, 0)

        # 断开连接
        await controller.disconnect()

# 运行主函数
asyncio.run(main())
```

## 接口说明

### 初始化参数

```python
controller = RobotDogController(
    ip_address,                   # 机器人IP地址
    port=9090,                    # WebSocket端口
    max_linear_speed=0.1,         # 最大线速度
    max_angular_speed=0.2,        # 最大角速度
    acceleration_time=1.0,        # 加速时间(秒)
    deceleration_time=0.5,        # 减速时间(秒)
    ping_interval=30,             # 心跳间隔(秒)
    ping_timeout=5,               # 心跳超时时间(秒)
    update_interval=0.05          # 速度更新间隔(秒)
)
```

### 连接与断开

```python
await controller.connect()      # 连接到机器人，返回True/False表示成功/失败
await controller.disconnect()   # 断开连接
```

### 速度控制

```python
# 发送速度指令
await controller.send_speed(forward_speed, lateral_speed, angular_speed)
```

参数说明：
- `forward_speed`: 前进/后退速度 (正数为前进，负数为后退)
- `lateral_speed`: 左右移动速度 (正数为左移，负数为右移)
- `angular_speed`: 旋转速度 (正数为左转，负数为右转)

速度控制支持渐进式加减速，确保机器人运动平稳。

### 姿态控制

```python
await controller.toggle_posture()  # 切换站立/趴下状态
await controller.reset_pose()      # 回零姿态
await controller.emergency_stop()  # 软急停
```

### 模式切换

```python
await controller.switch_to_auto_mode()    # 切换到自动模式
await controller.switch_to_manual_mode()  # 切换到手动模式
```

### 参数设置

```python
controller.set_acceleration_time(seconds)  # 设置加速时间
controller.set_deceleration_time(seconds)  # 设置减速时间
controller.set_ping_interval(seconds)      # 设置心跳间隔
controller.set_ping_timeout(seconds)       # 设置心跳超时时间
```

### 事件订阅

```python
# 连接成功事件
controller.subscribe_on_connected(callback)
controller.unsubscribe_on_connected(callback)

# 断开连接事件
controller.subscribe_on_disconnected(callback)
controller.unsubscribe_on_disconnected(callback)

# 错误事件 (包括心跳超时)
controller.subscribe_on_error(callback)
controller.unsubscribe_on_error(callback)
```

错误事件回调函数接收一个包含错误信息的字典参数：`{"message": "错误信息"}`

## 示例：键盘控制

`robot_dog_controller_example.py` 提供了一个使用键盘控制机器狗的完整示例：

```python
from robot_dog_controller import RobotDogController
import asyncio
import keyboard

async def main():
    # 创建控制器实例
    controller = RobotDogController(
        ip_address="192.168.1.100",
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

    # 键盘控制映射
    print("按键说明:")
    print("  W/S: 前进/后退")
    print("  A/D: 左移/右移")
    print("  Q/E: 左转/右转")
    print("  Space: 站立/趴下切换")
    print("  R: 回零姿态")
    print("  P: 软急停")
    print("  1/2: 自动/手动模式")
    print("  Ctrl+C: 退出程序")

    try:
        while True:
            # 处理方向控制
            forward_speed = 0.0
            lateral_speed = 0.0
            angular_speed = 0.0

            if keyboard.is_pressed('w'):
                forward_speed = controller.max_linear_speed
            elif keyboard.is_pressed('s'):
                forward_speed = -controller.max_linear_speed

            if keyboard.is_pressed('a'):
                lateral_speed = controller.max_linear_speed
            elif keyboard.is_pressed('d'):
                lateral_speed = -controller.max_linear_speed

            if keyboard.is_pressed('q'):
                angular_speed = controller.max_angular_speed
            elif keyboard.is_pressed('e'):
                angular_speed = -controller.max_angular_speed

            # 发送速度指令
            await controller.send_speed(forward_speed, lateral_speed, angular_speed)

            # 处理功能按键
            if keyboard.is_pressed('space'):
                await controller.toggle_posture()
                await asyncio.sleep(0.2)  # 防止重复触发

            if keyboard.is_pressed('r'):
                await controller.reset_pose()
                await asyncio.sleep(0.2)

            if keyboard.is_pressed('p'):
                await controller.emergency_stop()
                await asyncio.sleep(0.2)

            if keyboard.is_pressed('1'):
                await controller.switch_to_auto_mode()
                await asyncio.sleep(0.2)

            if keyboard.is_pressed('2'):
                await controller.switch_to_manual_mode()
                await asyncio.sleep(0.2)

            # 控制循环频率
            await asyncio.sleep(0.05)

    except KeyboardInterrupt:
        print("\n程序已退出")
    finally:
        await controller.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
```

## 注意事项

1. 所有控制方法都是异步的，需要使用`await`调用
2. 速度控制支持平滑加减速，避免机器人运动突变
3. 确保在使用结束后调用`disconnect()`断开连接
4. 控制器内部实现了心跳机制，可以自动检测连接状态
5. 出现错误时会通过错误事件通知，可以注册回调函数处理

## 高级使用

### 自定义心跳参数

对于不稳定的网络环境，可以调整心跳参数：

```python
controller = RobotDogController(
    ip_address="192.168.1.100",
    ping_interval=1.0,    # 更频繁地发送心跳
    ping_timeout=3.0      # 更短的超时时间
)
```

### 调整加减速参数

根据实际需求调整加减速参数，使机器人运动更平滑或更灵敏：

```python
controller = RobotDogController(
    ip_address="192.168.1.100",
    acceleration_time=0.5,    # 加速更快
    deceleration_time=0.3     # 减速更快
)
```
