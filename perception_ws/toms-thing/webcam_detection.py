import cv2
import tkinter as tk
from ultralytics import YOLO

# Load model
model = YOLO("yolo26n.pt")

# Open webcam
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    raise RuntimeError("Could not open webcam")

# Get screen resolution
root = tk.Tk()
root.withdraw()
SCREEN_WIDTH = root.winfo_screenwidth()
SCREEN_HEIGHT = root.winfo_screenheight()
root.destroy()

window_name = "Self-Driving Perception Demo"

cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(
    window_name,
    cv2.WND_PROP_FULLSCREEN,
    cv2.WINDOW_FULLSCREEN
)


def fit_to_screen(frame, screen_width, screen_height):
    """
    Scale frame as large as possible without changing its aspect ratio.
    Any unused space becomes black.
    """
    height, width = frame.shape[:2]

    scale = min(
        screen_width / width,
        screen_height / height
    )

    new_width = int(width * scale)
    new_height = int(height * scale)

    resized = cv2.resize(
        frame,
        (new_width, new_height),
        interpolation=cv2.INTER_LINEAR
    )

    # Amount of black border needed
    left = (screen_width - new_width) // 2
    right = screen_width - new_width - left
    top = (screen_height - new_height) // 2
    bottom = screen_height - new_height - top

    return cv2.copyMakeBorder(
        resized,
        top,
        bottom,
        left,
        right,
        cv2.BORDER_CONSTANT,
        value=(0, 0, 0)
    )


while True:
    ok, frame = camera.read()

    if not ok:
        break

    results = model(
        frame,
        conf=0.40,
        verbose=False
    )

    # Draw detections first
    display_frame = results[0].plot()

    # Then scale everything to fullscreen
    display_frame = fit_to_screen(
        display_frame,
        SCREEN_WIDTH,
        SCREEN_HEIGHT
    )

    cv2.imshow(window_name, display_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


camera.release()
cv2.destroyAllWindows()