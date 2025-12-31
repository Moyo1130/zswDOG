from ultralytics import YOLO
import cv2

# Load your YOLO model
model = YOLO("yolo11n.pt")  # or yolov8n.pt if using YOLOv8

# RTSP source
source = "rtsp://admin:Zswbimvr@192.168.1.201:554/Streaming/Channels/101"

# Open RTSP stream with OpenCV
cap = cv2.VideoCapture(source)

if not cap.isOpened():
    print("❌ Could not open RTSP stream.")
    exit()

print("✅ Press 'q' to quit the stream.")

# Run YOLO on the stream
for result in model(source, stream=True):
    frame = result.plot()  # Annotated frame
    cv2.imshow("YOLO RTSP Stream", frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()