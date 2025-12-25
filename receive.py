import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def pose_callback(msg):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    # 从消息中提取位置和方向
    px = position.x
    py = position.y
    pz = position.z
    qx = orientation.x
    qy = orientation.y
    qz = orientation.z
    qw = orientation.w
    # 计算偏航角（例如）
    print(f"Position: x={px}, y={py}, z={pz}")
    print(f"Orientation: x={qx}, y={qy}, z={qz}, w={qw}")

def listener():
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber("/leg_odom", PoseWithCovarianceStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

