import cv2
import numpy as np

def init_camera(video_id=0):
    cap = cv2.VideoCapture(video_id)
    if not cap.isOpened():
        print(f"Could not open camera {video_id}")
        return None
    
    # Create a named window and set its size
    window_name = "frame"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 800, 600)
    return cap, window_name

def get_dominant_color(frame):
    # Convert frame to RGB color space
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Define a region of interest in the center of the frame
    height, width = frame.shape[:2]
    roi_size = 100
    x = width//2 - roi_size//2
    y = height//2 - roi_size//2
    roi = rgb_frame[y:y+roi_size, x:x+roi_size]
    
    # Draw rectangle to show ROI
    cv2.rectangle(frame, (x, y), (x+roi_size, y+roi_size), (0, 255, 0), 2)
    
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
    cap, window_name = init_camera()
    if cap is None:
        return None
    
    ret, frame = cap.read()
    if ret:
        color = get_dominant_color(frame)
        cv2.putText(frame, f"Color: {color}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow(window_name, frame)
        cv2.waitKey(1)  # Show frame briefly
        
        cap.release()
        cv2.destroyAllWindows()
        return color
    else:
        cap.release()
        cv2.destroyAllWindows()
        return None

if __name__ == "__main__":
    # Example usage
    color = get_color()
    print(f"Detected color: {color}")