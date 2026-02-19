from picamera2 import Picamera2
import cv2

picam2 = Picamera2()

# Video config tends to be the most predictable for OpenCV pipelines
config = picam2.create_video_configuration(
    main={"size": (1280, 720), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

# Grab one frame and print what we actually received
frame = picam2.capture_array("main")
print("Frame:", frame.shape, frame.dtype)

# Decide conversion once (lowest CPU approach)
# OpenCV expects BGR for display; Picamera2 main is often RGB888.
needs_rgb_to_bgr = True

while True:
    frame = picam2.capture_array("main")

    if frame.ndim == 3 and frame.shape[2] == 4:
        # If you ever get 4-channel frames, strip alpha correctly
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    elif needs_rgb_to_bgr:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    cv2.imshow("Cam", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
