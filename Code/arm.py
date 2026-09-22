import json
import time
import math
from pymavlink import mavutil
import arm_positioning as ap

CRUISE_ALTITUDE_M = 3.0
TAKEOFF_TIMEOUT_S = 30

GRIPPER_SERVO_INSTANCE = 9  # AUX1 (AUX outputs start at 9; AUX2=10, AUX3=11, etc.)
GRIPPER_OPEN_PWM = 1100
GRIPPER_CLOSED_PWM = 1900

TRASH_APPROACH_CONFIDENCE = 0.5
TRASH_GRIP_AREA_FRAC = 0.15
TRASH_CAN_CONFIDENCE = 0.5
BIN_RELEASE_AREA_FRAC = 0.25  # how large the bin opening must appear (bottom camera) before releasing — untuned placeholder

LOW_BATTERY_PERCENT = 20
MANUAL_CONTROL_TIMEOUT_S = 1.0  # if no manual command in this long, fall back to autonomous

STATE_COVERAGE = "COVERAGE"
STATE_APPROACH_TRASH = "APPROACH_TRASH"
STATE_GRIP = "GRIP"
STATE_DELIVER = "DELIVER"
STATE_RELEASE = "RELEASE"
STATE_RETURN_TO_LAUNCH = "RETURN_TO_LAUNCH"

state = STATE_COVERAGE
holding_trash = False


def connect():
    master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
    master.wait_heartbeat()
    print("Heartbeat received from system", master.target_system)
    return master


def set_ekf_origin(master):
    # No GPS, so the EKF has no reference point to start from until one is
    # given explicitly. The actual lat/lon values don't matter for our
    # purposes — only the local NED offsets from this origin (which come
    # from our own vision position estimates) are ever used for control.
    master.mav.set_gps_global_origin_send(
        master.target_system,
        0,      # latitude, degE7 — arbitrary, only a reference point
        0,      # longitude, degE7 — arbitrary
        0,      # altitude, mm
        int(time.time() * 1e6)
    )
    print("Sent EKF origin")


def send_vision_position(master, mission_state):
    pose = mission_state["pose"]
    master.mav.vision_position_estimate_send(
        int(time.time() * 1e6),
        pose["x"], pose["y"], 0.0,
        0.0, 0.0, pose["theta"],
        [0.0] * 21,
        0
    )


def publish_pixhawk_telemetry(master):
    attitude_msg = master.recv_match(type='ATTITUDE', blocking=False)
    position_msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)

    if attitude_msg is not None and position_msg is not None:
        latest_pose = {
            'yaw': attitude_msg.yaw,
            'x': position_msg.x,
            'y': position_msg.y,
        }
        with open("/tmp/pixhawk_pose.json", "w") as f:
            json.dump(latest_pose, f)


def arm_and_takeoff(master, altitude_m):
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Armed")

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude_m
    )

    start_time = time.time()
    while time.time() - start_time < TAKEOFF_TIMEOUT_S:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
        if msg is not None and -msg.z >= altitude_m * 0.9:
            print("Reached cruise altitude")
            return True
    print("Takeoff timed out — did not confirm cruise altitude")
    return False


def send_position_target(master, x, y, z_altitude_m):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        3576,  # position only, ignore velocity/accel/yaw/yaw_rate
        x, y, -z_altitude_m,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def set_gripper(master, pwm):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        GRIPPER_SERVO_INSTANCE, pwm,
        0, 0, 0, 0, 0
    )


def open_gripper(master):
    set_gripper(master, GRIPPER_OPEN_PWM)


def close_gripper(master):
    set_gripper(master, GRIPPER_CLOSED_PWM)


def move_shoulder(master, angle_deg):
    pwm = ap.angle_to_pwm(
        angle_deg,
        ap.SHOULDER_PWM_AT_MIN_ANGLE, ap.SHOULDER_PWM_AT_MAX_ANGLE,
        ap.SHOULDER_MIN_ANGLE_DEG, ap.SHOULDER_MAX_ANGLE_DEG
    )
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        ap.SHOULDER_SERVO_INSTANCE, pwm,
        0, 0, 0, 0, 0
    )


def move_wrist(master, angle_deg):
    pwm = ap.angle_to_pwm(
        angle_deg,
        ap.WRIST_PWM_AT_MIN_ANGLE, ap.WRIST_PWM_AT_MAX_ANGLE,
        ap.WRIST_MIN_ANGLE_DEG, ap.WRIST_MAX_ANGLE_DEG
    )
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        ap.WRIST_SERVO_INSTANCE, pwm,
        0, 0, 0, 0, 0
    )


def fold_arm(master):
    """Fold the arm up as far as physically possible — used before takeoff
    and after landing, so it's stowed whenever the drone isn't actively
    using it."""
    move_shoulder(master, ap.SHOULDER_STOWED_ANGLE_DEG)
    move_wrist(master, ap.WRIST_STOWED_ANGLE_DEG)
    print("Arm folded")


def read_json(path):
    try:
        with open(path, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def get_battery_percent(master):
    msg = master.recv_match(type='SYS_STATUS', blocking=False)
    if msg is not None and msg.battery_remaining >= 0:
        return msg.battery_remaining
    return None


def blocked_toward(dx, dy, nearby):
    if nearby is None:
        return False
    if dx > 0 and nearby.get("front"):
        return True
    if dx < 0 and nearby.get("back"):
        return True
    if dy > 0 and nearby.get("right"):
        return True
    if dy < 0 and nearby.get("left"):
        return True
    return False


def run():
    global state, holding_trash

    master = connect()
    fold_arm(master)
    set_ekf_origin(master)

    print("Waiting for SLAM pose before arming...")
    mission_state = None
    while mission_state is None:
        publish_pixhawk_telemetry(master)
        mission_state = read_json("/tmp/mission_state.json")
        if mission_state is not None:
            send_vision_position(master, mission_state)
        time.sleep(0.1)
    print("SLAM pose available — proceeding")

    if not arm_and_takeoff(master, CRUISE_ALTITUDE_M):
        print("Aborting — takeoff failed")
        return

    rtl_commanded = False

    try:
        while True:
            publish_pixhawk_telemetry(master)

            emergency_command = read_json("/tmp/emergency_command.json")
            if emergency_command is not None and emergency_command.get("command") == "rtl":
                state = STATE_RETURN_TO_LAUNCH

            mission_state = read_json("/tmp/mission_state.json")
            trash_data = read_json("/tmp/trash_detection.json")
            front_data = read_json("/tmp/front_detection.json")

            if mission_state is not None:
                send_vision_position(master, mission_state)

            battery_percent = get_battery_percent(master)
            if battery_percent is not None and battery_percent < LOW_BATTERY_PERCENT:
                state = STATE_RETURN_TO_LAUNCH

            if mission_state is None:
                time.sleep(0.1)
                continue

            pose = mission_state["pose"]
            nearby = mission_state.get("nearby")

            manual_command = read_json("/tmp/manual_control.json")
            manual_active = (
                manual_command is not None and
                (time.time() - manual_command.get("timestamp", 0)) < MANUAL_CONTROL_TIMEOUT_S
            )

            if state == STATE_RETURN_TO_LAUNCH:
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                print("Returning to launch — battery low")
                rtl_commanded = True
                break

            elif manual_active:
                dx = manual_command.get("dx", 0)
                dy = manual_command.get("dy", 0)
                nudge = 0.5  # meters per command — untuned placeholder, adjust once tested
                target_x = pose["x"] + dx * nudge
                target_y = pose["y"] + dy * nudge
                if not blocked_toward(dx, dy, nearby):
                    send_position_target(master, target_x, target_y, CRUISE_ALTITUDE_M)
                else:
                    print("Manual move blocked by nearby obstacle — holding position")
                    send_position_target(master, pose["x"], pose["y"], CRUISE_ALTITUDE_M)

            elif state == STATE_COVERAGE:
                best_trash = None
                if trash_data and trash_data["detections"]:
                    top = trash_data["detections"][0]
                    if top["confidence"] >= TRASH_APPROACH_CONFIDENCE:
                        best_trash = top

                if best_trash is not None:
                    state = STATE_APPROACH_TRASH
                    print("Trash spotted — switching to approach")
                else:
                    target = mission_state.get("next_target")
                    if target is not None:
                        dx = target["x"] - pose["x"]
                        dy = target["y"] - pose["y"]
                        if not blocked_toward(dx, dy, nearby):
                            send_position_target(master, target["x"], target["y"], CRUISE_ALTITUDE_M)
                        else:
                            print("Path to coverage target blocked — holding position")
                            send_position_target(master, pose["x"], pose["y"], CRUISE_ALTITUDE_M)

            elif state == STATE_APPROACH_TRASH:
                if not trash_data or not trash_data["detections"]:
                    state = STATE_COVERAGE
                else:
                    top = trash_data["detections"][0]
                    offset_x = (top["center_x_frac"] - 0.5)
                    offset_y = (top["center_y_frac"] - 0.5)

                    if top["box_area_frac"] >= TRASH_GRIP_AREA_FRAC:
                        state = STATE_GRIP
                    else:
                        nudge_scale = 0.3
                        target_x = pose["x"] - offset_y * nudge_scale
                        target_y = pose["y"] + offset_x * nudge_scale
                        send_position_target(master, target_x, target_y, CRUISE_ALTITUDE_M)

            elif state == STATE_GRIP:
                send_position_target(master, pose["x"], pose["y"], CRUISE_ALTITUDE_M)
                close_gripper(master)
                time.sleep(2)
                holding_trash = True
                state = STATE_DELIVER
                print("Gripped — heading to deliver")

            elif state == STATE_DELIVER:
                # Prefer the bottom camera once it can see the bin — that
                # means the drone is close enough/positioned above it,
                # which is exactly when precise alignment matters most.
                # The front camera (forward-facing) can't see the opening
                # at that point at all, so it's only useful for the rough,
                # farther-out approach.
                bottom_can = None
                if trash_data and trash_data.get("trash_cans"):
                    top = trash_data["trash_cans"][0]
                    if top["confidence"] >= TRASH_CAN_CONFIDENCE:
                        bottom_can = top

                if bottom_can is not None:
                    if bottom_can["box_area_frac"] >= BIN_RELEASE_AREA_FRAC:
                        state = STATE_RELEASE
                    else:
                        offset_x = (bottom_can["center_x_frac"] - 0.5)
                        offset_y = (bottom_can["center_y_frac"] - 0.5)
                        nudge_scale = 0.15  # smaller than the front-camera nudge — this is fine alignment
                        target_x = pose["x"] - offset_y * nudge_scale
                        target_y = pose["y"] + offset_x * nudge_scale
                        send_position_target(master, target_x, target_y, CRUISE_ALTITUDE_M)
                else:
                    front_can = None
                    if front_data and front_data.get("trash_cans"):
                        top = front_data["trash_cans"][0]
                        if top["confidence"] >= TRASH_CAN_CONFIDENCE:
                            front_can = top

                    if front_can is not None:
                        offset_x = (front_can["center_x_frac"] - 0.5)
                        offset_y = (front_can["center_y_frac"] - 0.5)
                        nudge_scale = 0.3
                        target_x = pose["x"] - offset_y * nudge_scale
                        target_y = pose["y"] + offset_x * nudge_scale
                        send_position_target(master, target_x, target_y, CRUISE_ALTITUDE_M)
                    else:
                        send_position_target(master, pose["x"], pose["y"], CRUISE_ALTITUDE_M)

            elif state == STATE_RELEASE:
                open_gripper(master)
                time.sleep(2)
                holding_trash = False
                state = STATE_COVERAGE
                print("Released — resuming coverage")

            time.sleep(0.2)

    finally:
        if not rtl_commanded:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            print("Landing")
        else:
            print("RTL already commanded — letting it fly home and land on its own")

        fold_arm(master)


if __name__ == "__main__":
    run()