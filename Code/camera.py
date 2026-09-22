from picamera2 import Picamera2
from ultralytics import YOLO
import numpy as np
import cv2
import json
import time

# General pretrained model, still used only for trash-can detection
# (no bin-type-specific model trained yet).
GENERAL_MODEL_PATH = "yolov8n-oiv7.pt"

# Real trained trash detector (from train_trash_model.py, TACO dataset).
# On the Pi, this should point at the copied-over best_ncnn_model folder,
# e.g. "best_ncnn_model" — update this path to wherever you place it.
TRASH_MODEL_PATH = "best_ncnn_model"

TRASH_CAN_CLASS_NAMES = {"waste container"}

CONFIDENCE_THRESHOLD = 0.4

front_camera = Picamera2(0)
bottom_camera = Picamera2(1)

front_camera.start()
bottom_camera.start()

general_model = YOLO(GENERAL_MODEL_PATH)
trash_model = YOLO(TRASH_MODEL_PATH)


def detect_objects(frame, model, class_names=None, confidence_threshold=CONFIDENCE_THRESHOLD):
    """Run a model on a frame and return matching detections.
    class_names=None means accept every class the model knows — appropriate
    for the trash model, where every class genuinely is litter. Pass an
    explicit set to filter, as the trash-can detector still needs to."""
    results = model.predict(source=frame, verbose=False)[0]
    frame_height, frame_width = frame.shape[:2]

    detections = []
    for box in results.boxes:
        class_id = int(box.cls[0])
        class_name = model.names[class_id]
        confidence = float(box.conf[0])

        if confidence < confidence_threshold:
            continue
        if class_names is not None and class_name not in class_names:
            continue

        x1, y1, x2, y2 = box.xyxy[0].tolist()
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2

        detections.append({
            "class_name": class_name,
            "confidence": confidence,
            "center_x_frac": center_x / frame_width,
            "center_y_frac": center_y / frame_height,
            "box_area_frac": ((x2 - x1) * (y2 - y1)) / (frame_width * frame_height)
        })

    detections.sort(key=lambda d: d["confidence"], reverse=True)
    return detections


def detect_water_heuristic(frame):
    """Crude interim water detector — no trained model/dataset exists for
    this yet (see earlier discussion: water isn't a good fit for object
    detection at all). This looks for unusually SMOOTH regions in the lower
    half of the frame (low local texture/contrast), on the idea that water
    reflects its surroundings and lacks the texture of grass/pavement/dirt.

    This WILL be wrong sometimes — wet pavement can look smooth too, and
    murky or heavily shadowed water may not. Treat "water_detected" as
    advisory, not a safety guarantee, until validated against real footage
    or replaced with a properly trained model."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    height = gray.shape[0]
    lower_region = gray[height // 2:, :]  # water would appear at/below the horizon in a forward view

    texture_variance = cv2.Laplacian(lower_region, cv2.CV_64F).var()

    WATER_TEXTURE_THRESHOLD = 50  # untuned placeholder — calibrate against real footage
    is_smooth = texture_variance < WATER_TEXTURE_THRESHOLD

    return {
        "water_detected": bool(is_smooth),
        "texture_variance": float(texture_variance)  # exposed for tuning/debugging, not a confidence score
    }


def save_frame_for_streaming(frame, path):
    # Note: picamera2's default frame format may not be plain BGR — if
    # colors look wrong once streamed to the phone, this is the spot to
    # add a cv2.cvtColor conversion to match whatever format your camera
    # config actually produces.
    success, jpeg_bytes = cv2.imencode(".jpg", frame)
    if success:
        with open(path, "wb") as f:
            f.write(jpeg_bytes.tobytes())


try:
    while True:
        bottom_frame = bottom_camera.capture_array()
        trash_detections = detect_objects(bottom_frame, trash_model)  # class_names=None: every class is litter

        # Bin detection ALSO runs on the bottom camera, not just the front —
        # needed for the final delivery approach, since once the drone is
        # actually hovering above a bin, the opening is below it, out of
        # the forward-facing front camera's view entirely. Same reasoning
        # as why trash pickup itself uses the bottom camera, not the front.
        bottom_trash_can_detections = detect_objects(bottom_frame, general_model, TRASH_CAN_CLASS_NAMES)
        save_frame_for_streaming(bottom_frame, "/tmp/bottom_frame.jpg")

        with open("/tmp/trash_detection.json", "w") as f:
            json.dump({
                "timestamp": time.time(),
                "detections": trash_detections,
                "trash_cans": bottom_trash_can_detections
            }, f)

        front_frame = front_camera.capture_array()
        trash_can_detections = detect_objects(front_frame, general_model, TRASH_CAN_CLASS_NAMES)
        water_result = detect_water_heuristic(front_frame)
        save_frame_for_streaming(front_frame, "/tmp/front_frame.jpg")

        with open("/tmp/front_detection.json", "w") as f:
            json.dump({
                "timestamp": time.time(),
                "trash_cans": trash_can_detections,
                "water": water_result
            }, f)

finally:
    front_camera.stop()
    bottom_camera.stop()