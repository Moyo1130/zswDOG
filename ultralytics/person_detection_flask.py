import cv2
import time
import threading
import requests
from ultralytics import YOLO

# Configuration
MODEL_NAME = "yolo11n.pt"
RTSP_URL = "rtsp://admin:Zswbimvr@192.168.1.201:554/Streaming/Channels/101"
FLASK_BACKEND_URL = "http://127.0.0.1:5000/api/person_detected"  # Replace with your actual Flask endpoint
PAUSE_DURATION = 10  # Seconds to pause detection after finding a person

def send_signal_to_backend():
    """Sends a signal to the Flask backend in a separate thread."""
    try:
        # You can customize the payload as needed
        payload = {"event": "person_detected", "timestamp": time.time()}
        print(f"Sending signal to {FLASK_BACKEND_URL} with payload: {payload}")
        
        # Set a timeout to avoid hanging threads
        response = requests.post(FLASK_BACKEND_URL, json=payload, timeout=5)
        print(f"Backend response: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Failed to contact backend: {e}")

def main():
    # Load the YOLO model
    print(f"Loading model {MODEL_NAME}...")
    model = YOLO(MODEL_NAME)

    # Open the video stream
    print(f"Opening video stream {RTSP_URL}...")
    cap = cv2.VideoCapture(RTSP_URL)
    
    if not cap.isOpened():
        print(f"Error: Could not open video source {RTSP_URL}")
        return

    print("Starting detection loop. Press Ctrl+C to exit.")

    last_detection_time = 0
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to read frame from stream.")
                # Attempt to reconnect or break? For now, we break.
                break

            current_time = time.time()
            time_since_last_detection = current_time - last_detection_time

            # Check if we are in the cooldown period
            if time_since_last_detection > PAUSE_DURATION:
                # Run inference
                # verbose=False suppresses the default print output from YOLO
                results = model(frame, verbose=False)
                
                person_detected = False
                
                # Check results for 'person' class (class ID 0 for COCO dataset)
                # YOLO results list corresponds to the batch size (1 in this case)
                for result in results:
                    for box in result.boxes:
                        if int(box.cls) == 0:  # 0 is the class ID for person
                            person_detected = True
                            break
                    if person_detected:
                        break
                
                if person_detected:
                    print(f"Person detected! Pausing detection for {PAUSE_DURATION} seconds.")
                    last_detection_time = current_time
                    
                    # Send signal asynchronously so we don't block the video processing
                    threading.Thread(target=send_signal_to_backend, daemon=True).start()

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        cap.release()

if __name__ == "__main__":
    main()
