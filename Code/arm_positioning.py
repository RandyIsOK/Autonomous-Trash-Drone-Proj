import math

# ---------- ARM GEOMETRY — placeholders, replace with your real measurements ----------
SHOULDER_LINK_LENGTH_M = 0.20   # TODO: measure shoulder-to-wrist joint distance
WRIST_LINK_LENGTH_M = 0.10      # TODO: measure wrist-to-gripper-tip distance

# ---------- SERVO CALIBRATION — placeholders, replace after bench testing each servo ----------
SHOULDER_SERVO_INSTANCE = 10   # AUX2
WRIST_SERVO_INSTANCE = 11      # AUX3

SHOULDER_PWM_AT_MIN_ANGLE = 1100   # PWM when shoulder is at its most "folded in" angle
SHOULDER_PWM_AT_MAX_ANGLE = 1900   # PWM when shoulder is at its most "extended" angle
SHOULDER_MIN_ANGLE_DEG = 0
SHOULDER_MAX_ANGLE_DEG = 180

WRIST_PWM_AT_MIN_ANGLE = 1100
WRIST_PWM_AT_MAX_ANGLE = 1900
WRIST_MIN_ANGLE_DEG = 0
WRIST_MAX_ANGLE_DEG = 180

# ---------- STOWED POSITION — folded up as far as physically possible ----------
# TODO: these are placeholders (assuming "max angle" folds the arm inward) —
# once the real arm exists, jog each servo by hand/bench test to find its
# actual most-folded angle and replace these with the real values.
SHOULDER_STOWED_ANGLE_DEG = SHOULDER_MAX_ANGLE_DEG
WRIST_STOWED_ANGLE_DEG = WRIST_MAX_ANGLE_DEG

# ---------- BIN SIZES — placeholders, measure the real opening width of each bin type ----------
BIN_WIDTHS_M = {
    "waste container": 0.45,  # TODO: replace with real measured widths per bin type,
                               # and add entries here once bin-type classes exist
}

# ---------- CAMERA CALIBRATION — must be measured, not guessed from a spec sheet ----------
# To calibrate: place an object of known width at a known distance from the
# camera, measure its width in pixels in the captured frame, then:
#   FOCAL_LENGTH_PX = (pixel_width * distance_m) / real_width_m
FOCAL_LENGTH_PX = None  # TODO: calibrate — see comment above. Nothing below
                         # that uses this will produce a valid distance until set.


def inverse_kinematics(target_x, target_y):
    """2-link planar inverse kinematics: given a target (x, y) position
    relative to the shoulder joint's mounting point, return the
    (shoulder_angle_deg, wrist_angle_deg) needed to reach it, or None if
    the target is out of the arm's physical reach."""
    distance = math.hypot(target_x, target_y)
    max_reach = SHOULDER_LINK_LENGTH_M + WRIST_LINK_LENGTH_M
    min_reach = abs(SHOULDER_LINK_LENGTH_M - WRIST_LINK_LENGTH_M)

    if distance > max_reach or distance < min_reach:
        return None  # target unreachable — too far or too close

    # Law of cosines: wrist bend angle
    cos_wrist = (SHOULDER_LINK_LENGTH_M**2 + WRIST_LINK_LENGTH_M**2 - distance**2) / \
                (2 * SHOULDER_LINK_LENGTH_M * WRIST_LINK_LENGTH_M)
    cos_wrist = max(-1.0, min(1.0, cos_wrist))  # guard against float rounding
    wrist_angle = math.pi - math.acos(cos_wrist)

    # Shoulder angle: angle to target, adjusted by the triangle's internal angle
    angle_to_target = math.atan2(target_y, target_x)
    cos_shoulder_offset = (SHOULDER_LINK_LENGTH_M**2 + distance**2 - WRIST_LINK_LENGTH_M**2) / \
                          (2 * SHOULDER_LINK_LENGTH_M * distance)
    cos_shoulder_offset = max(-1.0, min(1.0, cos_shoulder_offset))
    shoulder_offset = math.acos(cos_shoulder_offset)
    shoulder_angle = angle_to_target - shoulder_offset

    return math.degrees(shoulder_angle), math.degrees(wrist_angle)


def angle_to_pwm(angle_deg, pwm_at_min, pwm_at_max, min_angle_deg, max_angle_deg):
    """Linear mapping from a joint angle to the PWM value that achieves it,
    using the two-point calibration (min angle -> its PWM, max angle -> its
    PWM) measured on the bench for that specific servo."""
    angle_deg = max(min_angle_deg, min(max_angle_deg, angle_deg))
    fraction = (angle_deg - min_angle_deg) / (max_angle_deg - min_angle_deg)
    return int(pwm_at_min + fraction * (pwm_at_max - pwm_at_min))


def estimate_distance(known_width_m, apparent_width_px, focal_length_px=FOCAL_LENGTH_PX):
    """Monocular distance estimate via similar triangles: how far away
    something must be to appear this wide in the frame, given its real
    width and the camera's calibrated focal length."""
    if focal_length_px is None:
        raise ValueError("FOCAL_LENGTH_PX has not been calibrated yet")
    if apparent_width_px <= 0:
        return None
    return (known_width_m * focal_length_px) / apparent_width_px


def estimate_bin_position(detection, frame_width_px, frame_height_px, focal_length_px=FOCAL_LENGTH_PX):
    """Given a bin detection (from camera.py's detect_objects output) and
    the frame dimensions, estimate the bin's (x, y) position relative to
    the camera, in meters. Returns None if the bin's class has no known
    width or the camera isn't calibrated yet."""
    class_name = detection["class_name"]
    if class_name not in BIN_WIDTHS_M:
        return None

    apparent_width_px = math.sqrt(detection["box_area_frac"] * frame_width_px * frame_height_px)
    distance_m = estimate_distance(BIN_WIDTHS_M[class_name], apparent_width_px, focal_length_px)
    if distance_m is None:
        return None

    # Lateral offset from center of frame, converted from a fraction to meters at that distance
    center_offset_frac = detection["center_x_frac"] - 0.5
    lateral_offset_m = center_offset_frac * distance_m  # small-angle approximation

    return distance_m, lateral_offset_m