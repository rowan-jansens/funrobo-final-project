import cv2
import numpy as np

def get_dominant_color(frame, roi_box):
    x, y, roi_size = roi_box
    roi = frame[y:y+roi_size, x:x+roi_size]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mean_hsv = np.mean(hsv.reshape(-1, 3), axis=0)
    h, s, v = mean_hsv

    # Debug: print values
    print(f"HSV mean: H={h:.1f}, S={s:.1f}, V={v:.1f}")

    if s < 40 or v < 40:
        return "unknown"

    if h < 15 or h > 160:
        return "red"
    elif 35 < h < 85:
        return "green"
    elif 90 < h < 150:
        return "blue"
    else:
        return "unknown"

def get_block_color():
    """Capture one frame from camera, return dominant color in preset ROI."""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open camera 0.")
        return "unknown"

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Failed to capture frame.")
        return "unknown"

    roi_size = 50
    offset_x = 30
    offset_y = 120
    height, width = frame.shape[:2]
    x = width // 2 - roi_size // 2 + offset_x
    y = height // 2 - roi_size // 2 + offset_y
    roi_box = (x, y, roi_size)

    return get_dominant_color(frame, roi_box)

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open camera 0.")
        return

    roi_size = 50
    offset_x = 30
    offset_y = 120

    print("Press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break

        height, width = frame.shape[:2]
        x = width // 2 - roi_size // 2 + offset_x
        y = height // 2 - roi_size // 2 + offset_y
        roi_box = (x, y, roi_size)

        # Draw ROI and show color label
        color = get_dominant_color(frame, roi_box)
        cv2.rectangle(frame, (x, y), (x + roi_size, y + roi_size), (0, 255, 0), 2)
        cv2.putText(frame, f"Color: {color}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('HSV Color Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

