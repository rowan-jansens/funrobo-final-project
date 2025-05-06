import cv2
import numpy as np

def get_dominant_color(frame, roi_box):
    """
    Determine the dominant color in a region of interest (ROI) within a frame.

    Args:
        frame (ndarray): BGR image frame from the camera.
        roi_box (tuple): (x, y, roi_size) defining the square ROI in the frame.

    Returns:
        str: "red", "green", "blue", or "unknown" based on HSV color thresholds.
    """
    x, y, roi_size = roi_box

    # Extract ROI from the frame
    roi = frame[y:y+roi_size, x:x+roi_size]

    # Convert the ROI from BGR to HSV color space
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Compute the mean HSV value across the ROI
    mean_hsv = np.mean(hsv.reshape(-1, 3), axis=0)
    h, s, v = mean_hsv

    # Debug print of HSV values
    print(f"HSV mean: H={h:.1f}, S={s:.1f}, V={v:.1f}")

    # Check for low saturation or brightness (likely black/gray/white)
    if s < 40 or v < 40:
        return "unknown"

    # Determine color based on hue ranges
    if h < 15 or h > 160:
        return "red"
    elif 35 < h < 85:
        return "green"
    elif 90 < h < 150:
        return "blue"
    else:
        return "unknown"

def get_block_color():
    """
    Capture one frame from the camera and return the dominant color
    in a preset ROI (used without live display).
    
    Returns:
        str: Detected color name.
    """
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open camera 0.")
        return "unknown"

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Failed to capture frame.")
        return "unknown"

    # Define the ROI near the center of the frame with an offset
    roi_size = 50
    offset_x = 30
    offset_y = 120
    height, width = frame.shape[:2]
    x = width // 2 - roi_size // 2 + offset_x
    y = height // 2 - roi_size // 2 + offset_y
    roi_box = (x, y, roi_size)

    # Get the dominant color from the ROI
    return get_dominant_color(frame, roi_box)

def main():
    """
    Main function to display live video feed with a marked ROI and
    label showing the detected color in real time.
    """
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

        # Calculate ROI position relative to the frame center
        height, width = frame.shape[:2]
        x = width // 2 - roi_size // 2 + offset_x
        y = height // 2 - roi_size // 2 + offset_y
        roi_box = (x, y, roi_size)

        # Determine the dominant color and annotate the frame
        color = get_dominant_color(frame, roi_box)
        cv2.rectangle(frame, (x, y), (x + roi_size, y + roi_size), (0, 255, 0), 2)
        cv2.putText(frame, f"Color: {color}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show the annotated frame
        cv2.imshow('HSV Color Detection', frame)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Run the live color detection if this script is executed directly
if __name__ == "__main__":
    main()
