import cv2
import numpy as np

def find_working_camera():
    # Try indices from 0 to 10
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret:
                return i
    return None

def init_camera(video_id=None):
    if video_id is None:
        video_id = find_working_camera()
        if video_id is None:
            print("No working camera found")
            return None
        print(f"Using camera index: {video_id}")
    
    cap = cv2.VideoCapture(video_id)
    if not cap.isOpened():
        print(f"Could not open camera {video_id}")
        return None
    
    return cap

def get_dominant_color(frame):
    # Convert frame to RGB color space
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Define a region of interest in the center of the frame
    height, width = frame.shape[:2]
    roi_size = 100
    x = width//2 - roi_size//2
    y = height//2 - roi_size//2
    roi = rgb_frame[y:y+roi_size, x:x+roi_size]
    
    # Calculate average color in ROI
    mean_color = np.mean(roi, axis=(0,1))
    
    # Define color ranges
    if max(mean_color) < 50:
        return "black"
    elif min(mean_color) > 200:
        return "white"
    elif mean_color[0] > max(mean_color[1], mean_color[2]):
        return "red"
    elif mean_color[1] > max(mean_color[0], mean_color[2]):
        return "green"
    elif mean_color[2] > max(mean_color[0], mean_color[1]):
        return "blue"
    else:
        return "unknown"

def get_color():
    cap = init_camera()
    if cap is None:
        return None
    
    ret, frame = cap.read()
    if ret:
        color = get_dominant_color(frame)
        cap.release()
        return color
    else:
        cap.release()
        return None

if __name__ == "__main__":
    # Example usage
    color = get_color()
    print(f"Detected color: {color}")