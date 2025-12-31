import socket

DOG_IP = "192.168.1.120"   # 机器狗 WiFi IP
DOG_PORT = 43893

print("Sending command to robot dog...")

cmd = "rostopic list"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(cmd.encode("utf-8"), (DOG_IP, DOG_PORT))

# 接收返回结果
data, addr = sock.recvfrom(65535)
print("DOG RETURN:\n", data.decode("utf-8"))
