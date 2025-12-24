import socket
import yaml

DOG_IP = "10.65.234.12"
DOG_PORT = 43894

cmd = "rostopic echo /leg_odom -n 1"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(cmd.encode("utf-8"), (DOG_IP, DOG_PORT))

data, addr = sock.recvfrom(65535)
text = data.decode("utf-8")

# rostopic echo 末尾有 '---'，需要去掉
text = text.replace('---', '').strip()

# 解析 YAML
msg = yaml.safe_load(text)

# 提取 position / orientation
position = msg["pose"]["pose"]["position"]
orientation = msg["pose"]["pose"]["orientation"]

px, py, pz = position["x"], position["y"], position["z"]
qx, qy, qz, qw = (
    orientation["x"],
    orientation["y"],
    orientation["z"],
    orientation["w"],
)

print("Position:")
print(f"x={px}, y={py}, z={pz}")

print("Orientation (quaternion):")
print(f"x={qx}, y={qy}, z={qz}, w={qw}")