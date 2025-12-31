# ros_send_flask.py
import requests
import json

DOG_IP = "192.168.1.120"   # 机器狗 WiFi IP
DOG_PORT = 5000  # Flask默认端口

print("Sending command to robot dog via Flask...")

cmd = "rostopic list"

try:
    # 发送POST请求到机器狗的Flask服务器
    response = requests.post(
        f"http://{DOG_IP}:{DOG_PORT}/command",
        json={"command": cmd},
        timeout=10
    )
    
    if response.status_code == 200:
        result = response.json()
        print("DOG RETURN:")
        print(result.get("result", "No result"))
        print(f"\nStatus: {result.get('status', 'unknown')}")
    else:
        print(f"Error: HTTP {response.status_code}")
        print(response.text)
        
except requests.exceptions.Timeout:
    print("Error: Request timeout")
except requests.exceptions.ConnectionError:
    print(f"Error: Cannot connect to {DOG_IP}:{DOG_PORT}")
except Exception as e:
    print(f"Error: {e}")