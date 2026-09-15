import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial
import logging
import os
from datetime import datetime
import threading
from collections import deque
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
FRAME_W, FRAME_H = 1280, 720
LEFT_REGION = (0, FRAME_W // 3)
CENTER_REGION = (FRAME_W // 3, 2 * FRAME_W // 3)
RIGHT_REGION = (2 * FRAME_W // 3, FRAME_W)
SERVO_LEFT = 30
SERVO_RIGHT = 150
SERVO_STRAIGHT = 90
MIN_TURN = 10
MAX_TURN = 60
SIZE_THRESHOLD_SMALL = 1000
SIZE_THRESHOLD_LARGE = 8000
BASE_SPEED = 140
STOP_SPEED = 0
lower_red1 = np.array([171, 57, 84])
upper_red1 = np.array([184, 255, 255])
lower_red2 = np.array([171, 57, 84])
upper_red2 = np.array([184, 255, 255])
lower_green = np.array([51, 75, 54])
upper_green = np.array([85, 255, 255])
lower_magenta = np.array([135, 70, 60])
upper_magenta = np.array([175, 255, 255])
LOGS_DIR = "/home/rpi5wro/wro_fe_project/wro_fe_25/obstacle_round_v2/logs"
os.makedirs(LOGS_DIR, exist_ok=True)
RUN_TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_FILE = os.path.join(LOGS_DIR, f"navigation_log_{RUN_TIMESTAMP}.txt")
VIDEO_FILE = os.path.join(LOGS_DIR, f"navigation_video_{RUN_TIMESTAMP}.avi")
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)
video_writer = None
RECORD_VIDEO = True
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (FRAME_W, FRAME_H), "format": 'RGB888'})
initial_yaw = 0
yaw = 0
imu_raw_yaw = None
imu_system_calibration = 0
imu_gyro_calibration = 0
imu_accel_calibration = 0
imu_mag_calibration = 0
last_imu_update = 0
straight_heading_reference = 0.0
front_dist, left_dist, right_dist = 100, 35, 35
last_turn_direction = ""
recovery_mode = False
last_seen_obstacles = {"red": [], "green": [], "timestamp": 0}
OBSTACLE_MEMORY_TIMEOUT = 0.0
post_obstacle_delay_active = False
post_obstacle_delay_start = 0
POST_OBSTACLE_DELAY_DURATION = 1.0
obstacle_count = 0
max_obstacles_per_segment = 6
in_turn_mode = False
turn_start_time = 0
target_yaw = 0
TURN_TIMEOUT = 8.0
turn_just_completed = True
obstacle_deflection_after_turn = False
total_turns_completed = 0
MAX_TURNS_BEFORE_STOP = 12
navigation_mode = "WAITING"
parking_phase = "NONE"
parking_side = None
parking_start_heading = 0.0
parking_final_heading = 0.0
parking_phase_start = 0.0
parking_detection_streak = 0
parking_verified = False
PARKING_SPEED = 85
PARKING_REVERSE_SPEED = 75
PARKING_EXIT_ANGLE = 32.0
PARKING_REVERSE_ANGLE = 38.0
PARKING_HEADING_TOLERANCE = 4.0
PARKING_APPROACH_SIDE_DISTANCE = 32.0
PARKING_INSIDE_SIDE_DISTANCE = 22.0
PARKING_CLOSE_BLOCK_AREA = 2200
PARKING_CLOSE_BLOCK_BOTTOM = int(FRAME_H * 0.67)
PARKING_PASS_BLOCK_DURATION = 0.75
PARKING_FINAL_REVERSE_DURATION = 0.35
PARKING_PHASE_TIMEOUT = 4.0
turn_phase = "NONE"
turn_type = "MULTI_PHASE"
forward_phase_duration = 0.8
reverse_turn_duration = 3.0
backward_post_arc_duration = 0.8
backward_post_arc_start = 0
FRONT_DISTANCE_TURN_THRESHOLD = 60
SIDE_DISTANCE_THRESHOLD = 90
serial_lock = threading.Lock()
command_queue = deque(maxlen=3)
last_command_time = 0
COMMAND_INTERVAL = 0.05
sensor_data_fresh = False
last_sensor_update = 0
esp_start_event = threading.Event()
def serial_communication_handler():
    """Background thread to handle serial communication without blocking"""
    global front_dist, left_dist, right_dist, sensor_data_fresh, last_sensor_update
    global imu_raw_yaw, imu_system_calibration, imu_gyro_calibration
    global imu_accel_calibration, imu_mag_calibration, last_imu_update
    while True:
        try:
            with serial_lock:
                if ser.in_waiting > 0:
                    line = ser.readline().decode().strip()
                    if line == "START":
                        esp_start_event.set()
                        logger.info("START signal received from ESP32")
                    elif line.startswith("DATA"):
                        parts = line.split(",")
                        if len(parts) == 9:
                            _, f, l, r, raw_yaw, cal_sys, cal_gyro, cal_accel, cal_mag = parts
                            front_dist = int(float(f))
                            left_dist = int(float(l))
                            right_dist = int(float(r))
                            parsed_yaw = float(raw_yaw)
                            if 0.0 <= parsed_yaw < 360.0:
                                imu_raw_yaw = parsed_yaw
                                last_imu_update = time.time()
                            imu_system_calibration = int(cal_sys)
                            imu_gyro_calibration = int(cal_gyro)
                            imu_accel_calibration = int(cal_accel)
                            imu_mag_calibration = int(cal_mag)
                            sensor_data_fresh = True
                            last_sensor_update = time.time()
                            if not esp_start_event.is_set():
                                esp_start_event.set()
                                logger.info("ESP32 sensor stream started; using it as START confirmation")
                            logger.debug(
                                f"ESP sensor update: F={front_dist}, L={left_dist}, R={right_dist}, "
                                f"Yaw={imu_raw_yaw}, Cal={imu_system_calibration}/"
                                f"{imu_gyro_calibration}/{imu_accel_calibration}/{imu_mag_calibration}"
                            )
                    elif line == "ERROR_BNO055_NOT_FOUND":
                        logger.error("ESP32 could not find the BNO055")
                current_time = time.time()
                if (command_queue and
                    current_time - last_command_time > COMMAND_INTERVAL):
                    command = command_queue.popleft()
                    ser.write(command.encode())
                    ser.flush()
                    globals()['last_command_time'] = current_time
                    logger.debug(f"Sent command: {command.strip()}")
        except Exception as e:
            logger.warning(f"Serial communication error: {e}")
            time.sleep(0.01)
        time.sleep(0.005)
def send_to_esp32_improved(speed, steering):
    """Improved non-blocking command sending with queue management"""
    global last_command_time
    command = f"{int(speed)},{int(steering)}\n"
    with serial_lock:
        if not command_queue or command != list(command_queue)[-1]:
            if len(command_queue) >= command_queue.maxlen:
                removed = command_queue.popleft()
                logger.debug(f"Queue full, removed: {removed.strip()}")
            command_queue.append(command)
            logger.debug(f"Command queued: {command.strip()} (Queue size: {len(command_queue)})")
def get_fresh_sensor_data():
    """Get sensor data and return freshness status"""
    global sensor_data_fresh
    data_age = time.time() - last_sensor_update
    is_fresh = sensor_data_fresh and data_age < 0.5
    if sensor_data_fresh:
        sensor_data_fresh = False
    return front_dist, left_dist, right_dist, is_fresh, data_age
def should_initiate_turn(front_d, left_d, right_d, red_blocks, green_blocks):
    """Determine if a turn should be initiated based on conditions - VERY CONSERVATIVE"""
    global obstacle_count, turn_just_completed, obstacle_deflection_after_turn
    no_obstacles_at_all = len(red_blocks) == 0 and len(green_blocks) == 0
    if not no_obstacles_at_all:
        logger.debug(f"Turn BLOCKED: Obstacles still present - RED:{len(red_blocks)}, GREEN:{len(green_blocks)}")
        return None
    if turn_just_completed and not obstacle_deflection_after_turn:
        logger.debug(f"Turn BLOCKED: Turn completed but no active obstacle deflection occurred yet.")
        logger.debug(f"Sequence state: turn_just_completed={turn_just_completed}, obstacle_deflection_after_turn={obstacle_deflection_after_turn}")
        print("SEQUENCE CONSTRAINT: Must perform ACTIVE obstacle deflection before next turn!")
        return None
    elif turn_just_completed and obstacle_deflection_after_turn:
        logger.debug(f"Turn ALLOWED: Turn completed and obstacle deflection occurred.")
        print("SEQUENCE OK: Active obstacle deflection completed - turn now allowed")
    else:
        logger.debug(f"Turn ALLOWED: Initial state - no previous turn to constrain.")
        print("SEQUENCE OK: No previous turn constraint")
    front_blocked = front_d < FRONT_DISTANCE_TURN_THRESHOLD
    turn_right_condition = left_d < SIDE_DISTANCE_THRESHOLD and right_d > SIDE_DISTANCE_THRESHOLD
    turn_left_condition = left_d > SIDE_DISTANCE_THRESHOLD and right_d < SIDE_DISTANCE_THRESHOLD
    max_obstacles_reached = obstacle_count >= max_obstacles_per_segment
    logger.debug(f"Turn evaluation: NO_OBSTACLES={no_obstacles_at_all}, front_blocked={front_blocked}, "
                f"obstacles_handled={obstacle_count}/{max_obstacles_per_segment}, "
                f"distances(F:{front_d}, L:{left_d}, R:{right_d})")
    clear_turn_direction = turn_right_condition or turn_left_condition
    if front_blocked and clear_turn_direction:
        logger.info(f"TURN INITIATED: Front blocked ({front_d}) with clear path")
        if turn_right_condition:
            return "RIGHT"
        elif turn_left_condition:
            return "LEFT"
    if max_obstacles_reached and clear_turn_direction:
        logger.info(f"TURN INITIATED: Max obstacles ({obstacle_count}) reached with clear path")
        if turn_right_condition:
            return "RIGHT"
        elif turn_left_condition:
            return "LEFT"
    if not clear_turn_direction:
        logger.debug(f"Turn blocked: No clear direction - L:{left_d}, R:{right_d}")
    return None
def execute_turn(direction):
    """Execute adaptive turn based on ultrasonic readings: multi-phase or arc turn"""
    global in_turn_mode, turn_start_time, target_yaw, yaw, obstacle_count, turn_phase, turn_type
    global turn_just_completed, obstacle_deflection_after_turn, backward_post_arc_start
    if in_turn_mode:
        return False
    f_dist, l_dist, r_dist, data_fresh, data_age = get_fresh_sensor_data()
    if direction == "RIGHT":
        opposite_reading = l_dist
        opposite_side = "LEFT"
    else:
        opposite_reading = r_dist
        opposite_side = "RIGHT"
    if opposite_reading > 40:
        turn_type = "MULTI_PHASE"
        turn_description = "multi-phase (forward + reverse)"
    else:
        turn_type = "ARC"
        turn_description = "arc turn"
    print(f"INITIATING {direction} TURN: {opposite_side} ultrasonic = {opposite_reading}cm → {turn_description}")
    logger.info(f"Starting {direction} turn - {opposite_side} reading: {opposite_reading}cm → {turn_type} turn")
    turn_just_completed = False
    obstacle_deflection_after_turn = False
    in_turn_mode = True
    turn_start_time = time.time()
    current_yaw = read_yaw()
    print(f"=== UNBOUNDED TURN CALCULATION ===")
    print(f"Current Yaw: {current_yaw:.1f} deg")
    print(f"Turn Direction: {direction}")
    if direction == "RIGHT":
        target_yaw = current_yaw + 90.0
        print(f"RIGHT turn: {current_yaw:.1f} deg + 90.0 deg = {target_yaw:.1f} deg")
    else:
        target_yaw = current_yaw - 90.0
        print(f"LEFT turn: {current_yaw:.1f} deg - 90.0 deg = {target_yaw:.1f} deg")
    print(f"Final Target: {target_yaw:.1f} deg (NO normalization)")
    print(f"Expected angular change: 90.0 deg")
    print(f"=== END DEBUG ===")
    expected_diff = 90.0
    actual_diff = abs(target_yaw - current_yaw)
    if abs(actual_diff - expected_diff) > 1.0:
        print(f"ERROR: Turn calculation wrong! Expected 90 deg, got {actual_diff:.1f} deg")
        logger.error(f"Turn calculation error: {direction} from {current_yaw:.1f} deg should be 90 deg, got {actual_diff:.1f} deg")
    else:
        print(f"CORRECT: Turn calculation validated - {actual_diff:.1f} deg difference")
    print(f"TARGET YAW: Current={current_yaw:.1f} deg, Target={target_yaw:.1f} deg")
    logger.info(f"Target yaw calculation: Current={current_yaw:.1f}, Direction={direction}, Final target={target_yaw:.1f}")
    if turn_type == "MULTI_PHASE":
        turn_phase = "FORWARD"
        print(f"MULTI-PHASE: Going forward for {forward_phase_duration}s before reverse turn")
        logger.info(f"Multi-phase turn: Current yaw={current_yaw:.1f}, Target yaw={target_yaw:.1f}")
    else:
        turn_phase = "ARC"
        print(f"ARC TURN: Direct turning maneuver")
        logger.info(f"Arc turn: Current yaw={current_yaw:.1f}, Target yaw={target_yaw:.1f}")
    obstacle_count = 0
    return True
def update_turn_progress():
    """Update turn progress and return speed/steering commands for both multi-phase and arc turns"""
    global in_turn_mode, target_yaw, yaw, turn_phase, turn_type, backward_post_arc_start
    if not in_turn_mode:
        return None, None
    current_time = time.time()
    turn_duration = current_time - turn_start_time
    if turn_duration > TURN_TIMEOUT:
        print("WARNING: Turn timeout - aborting without counting the corner")
        logger.warning("Turn timeout reached; corner will not be counted")
        complete_turn(count_as_completed=False)
        return BASE_SPEED, SERVO_STRAIGHT
    current_yaw = read_yaw()
    if turn_type == "MULTI_PHASE":
        if turn_phase == "FORWARD":
            if turn_duration < forward_phase_duration:
                print(f"MULTI-PHASE FORWARD: ({turn_duration:.1f}/{forward_phase_duration:.1f}s)")
                return BASE_SPEED, SERVO_STRAIGHT
            else:
                turn_phase = "REVERSE_TURN"
                print(f"MULTI-PHASE REVERSE: Starting reverse turn phase (target: {target_yaw:.1f} deg)")
                logger.info(f"Transitioning to reverse turn phase - target yaw: {target_yaw:.1f}")
        if turn_phase == "REVERSE_TURN":
            error = debug_angle_calculation(current_yaw, target_yaw)
            if not hasattr(update_turn_progress, 'last_yaw_debug'):
                update_turn_progress.last_yaw_debug = current_yaw
                update_turn_progress.same_yaw_count = 0
            else:
                if abs(current_yaw - update_turn_progress.last_yaw_debug) < 0.5:
                    update_turn_progress.same_yaw_count += 1
                    if update_turn_progress.same_yaw_count >= 10:
                        print(f"YAW STUCK WARNING: Yaw has been {current_yaw:.1f} deg for {update_turn_progress.same_yaw_count} consecutive readings!")
                        logger.warning(f"Yaw stuck at {current_yaw:.1f} deg for {update_turn_progress.same_yaw_count} readings - IMU may be filtering valid changes")
                        if hasattr(read_yaw, 'stuck_count'):
                            print(f"FORCING IMU UPDATE: Resetting stuck counter from {read_yaw.stuck_count}")
                            read_yaw.stuck_count = 0
                else:
                    update_turn_progress.same_yaw_count = 0
                    if abs(current_yaw - update_turn_progress.last_yaw_debug) > 2.0:
                        print(f"YAW PROGRESSING: {update_turn_progress.last_yaw_debug:.1f} deg -> {current_yaw:.1f} deg (change: {current_yaw - update_turn_progress.last_yaw_debug:.1f} deg)")
                update_turn_progress.last_yaw_debug = current_yaw
            if abs(error) < 5.0:
                print(f"SUCCESS: Multi-phase turn completed! Final error: {error:.1f} deg")
                logger.info(f"Multi-phase turn completed - final error: {error:.1f} degrees")
                complete_turn()
                return BASE_SPEED, SERVO_STRAIGHT
            reverse_speed = -BASE_SPEED
            if error > 0:
                steering = SERVO_LEFT
                turn_desc = "RIGHT turn (LEFT steering in reverse)"
            else:
                steering = SERVO_RIGHT
                turn_desc = "LEFT turn (RIGHT steering in reverse)"
            if int(turn_duration * 4) % 3 == 0:
                logger.info(f"MULTI-PHASE PROGRESS: {turn_desc} | Current={current_yaw:.1f}deg | Target={target_yaw:.1f}deg | Error={error:.1f}deg")
                print(f"MULTI-PHASE: Current={current_yaw:.1f}deg, Target={target_yaw:.1f}deg, Error={error:.1f}deg, Duration={turn_duration:.1f}s")
            return reverse_speed, steering
    elif turn_type == "ARC":
        error = target_yaw - current_yaw
        if abs(error) < 5.0:
            print(f"SUCCESS: Arc turn completed! Error: {error:.1f} deg - Starting backward phase")
            logger.info(f"Arc turn completed successfully - final error: {error:.1f} deg - transitioning to backward phase")
            turn_phase = "BACKWARD_POST_ARC"
            backward_post_arc_start = time.time()
            print(f"ARC BACKWARD PHASE: Moving backward for {backward_post_arc_duration}s")
            logger.info(f"Arc turn backward phase started for {backward_post_arc_duration}s")
            return -BASE_SPEED + 20, SERVO_STRAIGHT
        speed = BASE_SPEED
        if error > 0:
            steering = SERVO_RIGHT
            turn_desc = "Arc RIGHT turn"
        else:
            steering = SERVO_LEFT
            turn_desc = "Arc LEFT turn"
        steering = max(SERVO_LEFT, min(SERVO_RIGHT, steering))
        if int(turn_duration * 2) % 2 == 0:
            logger.info(f"ARC PROGRESS: {turn_desc} | Current={current_yaw:.1f}deg | Target={target_yaw:.1f}deg | Error={error:.1f}deg")
            print(f"ARC: Current={current_yaw:.1f}deg, Target={target_yaw:.1f}deg, Error={error:.1f}deg")
        return speed, steering
    elif turn_phase == "BACKWARD_POST_ARC":
        backward_elapsed = current_time - backward_post_arc_start
        if backward_elapsed < backward_post_arc_duration:
            print(f"BACKWARD PHASE: {backward_elapsed:.1f}s / {backward_post_arc_duration}s")
            logger.debug(f"Arc backward phase: {backward_elapsed:.1f}s elapsed")
            return -BASE_SPEED + 20, SERVO_STRAIGHT
        else:
            print(f"BACKWARD PHASE COMPLETE: Arc turn fully completed after backward movement")
            logger.info(f"Arc backward phase completed - finishing turn")
            complete_turn()
            return BASE_SPEED, SERVO_STRAIGHT
    return BASE_SPEED, SERVO_STRAIGHT
def snap_to_cardinal_angle(current_angle):
    """Snap angle to nearest cardinal direction (0, 90, 180, 270, 360, etc.) to prevent error accumulation"""
    nearest_cardinal = round(current_angle / 90.0) * 90.0
    error = abs(current_angle - nearest_cardinal)
    if error <= 25.0:
        print(f"CARDINAL SNAP: {current_angle:.1f} deg -> {nearest_cardinal:.1f} deg (error: {error:.1f} deg)")
        logger.info(f"Cardinal angle correction: {current_angle:.1f} -> {nearest_cardinal:.1f} (error: {error:.1f})")
        return nearest_cardinal
    else:
        print(f"CARDINAL SKIP: {current_angle:.1f} deg error too large ({error:.1f} deg) - keeping original")
        logger.warning(f"Cardinal snap skipped: {current_angle:.1f} deg, error {error:.1f} deg too large")
        return current_angle
def complete_turn(count_as_completed=True):
    """Complete the current turn and reset turn mode with cardinal angle correction"""
    global in_turn_mode, obstacle_count, target_yaw, straight_heading_reference, turn_phase, turn_type
    global turn_just_completed, obstacle_deflection_after_turn, backward_post_arc_start
    global total_turns_completed, navigation_mode, parking_final_heading
    current_yaw_at_completion = read_yaw()
    old_reference = straight_heading_reference
    corrected_yaw = snap_to_cardinal_angle(current_yaw_at_completion)
    if hasattr(read_yaw, 'last_valid_yaw'):
        read_yaw.last_valid_yaw = corrected_yaw
    if hasattr(read_yaw, 'cumulative_yaw'):
        read_yaw.cumulative_yaw = corrected_yaw
    straight_heading_reference = corrected_yaw
    if count_as_completed:
        total_turns_completed += 1
    print(f"{turn_type} turn completed - CORRECTED straight reference: {straight_heading_reference:.1f} deg (was: {current_yaw_at_completion:.1f} deg)")
    print(f"REFERENCE UPDATE: Target was {target_yaw:.1f} deg, actual was {current_yaw_at_completion:.1f} deg, corrected to {corrected_yaw:.1f} deg")
    completion_label = "COUNTED" if count_as_completed else "NOT COUNTED"
    print(f"TURN COUNTER: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP} turns completed ({completion_label})")
    logger.info(f"{turn_type} turn completed - Updated straight reference from {old_reference:.1f} deg to {straight_heading_reference:.1f} deg (corrected from {current_yaw_at_completion:.1f})")
    logger.info(f"Turn completion: Target={target_yaw:.1f} deg, Actual={current_yaw_at_completion:.1f} deg, Corrected={corrected_yaw:.1f} deg")
    logger.info(f"Turn counter: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP} turns completed ({completion_label})")
    if count_as_completed and total_turns_completed >= MAX_TURNS_BEFORE_STOP:
        parking_final_heading = corrected_yaw
        navigation_mode = "FINAL_APPROACH"
        set_parking_phase("SEARCH_FOR_MAGENTA")
        print(f"THREE LAPS COMPLETE: Entering final parking approach at heading {parking_final_heading:.1f}")
        logger.info(f"Three laps complete; final parking approach heading={parking_final_heading:.1f}")
    in_turn_mode = False
    turn_phase = "NONE"
    turn_type = "MULTI_PHASE"
    obstacle_count = 0
    backward_post_arc_start = 0
    turn_just_completed = True
    obstacle_deflection_after_turn = False
    print("SEQUENCE CONTROL: Turn completed - Next turn blocked until obstacle deflection occurs")
    logger.info("Turn sequence control activated - next turn requires obstacle deflection first")
    if hasattr(update_turn_progress, 'last_yaw'):
        delattr(update_turn_progress, 'last_yaw')
    if hasattr(update_turn_progress, 'stuck_counter'):
        delattr(update_turn_progress, 'stuck_counter')
    time.sleep(0.1)
    print("TARGET: Turn sequence completed - resuming obstacle navigation with corrected heading")
    logger.info("Turn completed - resuming normal navigation with cardinal-corrected heading reference")
def log_navigation_data(steering, state, yaw, front_dist, left_dist, right_dist, red_blocks, green_blocks):
    """Log comprehensive navigation data"""
    timestamp = time.time()
    red_count = len(red_blocks)
    green_count = len(green_blocks)
    red_areas = [w * h for x, y, w, h in red_blocks] if red_blocks else []
    green_areas = [w * h for x, y, w, h in green_blocks] if green_blocks else []
    log_entry = (
        f"T:{timestamp:.3f} | Steering:{steering:.1f} | State:{state} | "
        f"Yaw:{yaw:.1f} | Dist(F:{front_dist},L:{left_dist},R:{right_dist}) | "
        f"RED_blocks:{red_count}{red_areas} | GREEN_blocks:{green_count}{green_areas}"
    )
    logger.info(log_entry)
def calibrate_imu():
    """Set the latest ESP32/BNO055 heading as the Pi's zero reference."""
    global initial_yaw, target_yaw, straight_heading_reference
    print("Waiting for BNO055 heading from ESP32...")
    deadline = time.time() + 5.0
    while imu_raw_yaw is None and time.time() < deadline:
        time.sleep(0.05)
    if imu_raw_yaw is None:
        raise RuntimeError("No valid BNO055 yaw received from ESP32 within 5 seconds")
    initial_yaw = imu_raw_yaw
    target_yaw = 0.0
    straight_heading_reference = 0.0
    for attribute in ("last_valid_yaw", "cumulative_yaw", "last_raw_yaw"):
        if hasattr(read_yaw, attribute):
            delattr(read_yaw, attribute)
    print(f"BNO055 heading received from ESP32. Initial heading: {initial_yaw:.1f} set as 0 reference")
    logger.info(
        f"ESP32 BNO055 zero set: raw={initial_yaw:.1f}, "
        f"calibration={imu_system_calibration}/{imu_gyro_calibration}/"
        f"{imu_accel_calibration}/{imu_mag_calibration}"
    )
def debug_angle_calculation(current_yaw, target_yaw):
    """Helper function to calculate turn error with NO normalization"""
    error = target_yaw - current_yaw
    print(f"UNBOUNDED ANGLE: Current={current_yaw:.1f} deg, Target={target_yaw:.1f} deg, Raw Error={error:.1f} deg")
    logger.debug(f"Unbounded angle calculation: current={current_yaw:.1f}, target={target_yaw:.1f}, error={error:.1f}")
    return error
def read_yaw():
    """Unwrap the latest BNO055 yaw received from the ESP32 into a cumulative heading."""
    global initial_yaw
    try:
        raw_yaw = imu_raw_yaw
        if raw_yaw is not None:
            relative_yaw = raw_yaw - initial_yaw
            if not hasattr(read_yaw, 'last_valid_yaw'):
                read_yaw.last_valid_yaw = relative_yaw
                read_yaw.cumulative_yaw = relative_yaw
                read_yaw.last_raw_yaw = raw_yaw
                print(f"IMU INIT: First reading = {relative_yaw:.1f} deg (raw: {raw_yaw:.1f} deg)")
                return relative_yaw
            raw_diff = raw_yaw - read_yaw.last_raw_yaw
            if raw_diff > 180:
                raw_diff -= 360
            elif raw_diff < -180:
                raw_diff += 360
            read_yaw.cumulative_yaw += raw_diff
            new_relative_yaw = read_yaw.cumulative_yaw
            change_from_last = abs(new_relative_yaw - read_yaw.last_valid_yaw)
            if change_from_last > 45:
                print(f"IMU REJECT: Too large change {change_from_last:.1f} deg - keeping {read_yaw.last_valid_yaw:.1f} deg")
                return read_yaw.last_valid_yaw
            if abs(new_relative_yaw - read_yaw.last_valid_yaw) > 2:
                print(f"IMU UPDATE: {read_yaw.last_valid_yaw:.1f} deg → {new_relative_yaw:.1f} deg (raw: {read_yaw.last_raw_yaw:.1f} deg → {raw_yaw:.1f} deg)")
            read_yaw.last_valid_yaw = new_relative_yaw
            read_yaw.last_raw_yaw = raw_yaw
            return new_relative_yaw
        else:
            logger.warning("No BNO055 yaw has been received from ESP32")
            if hasattr(read_yaw, 'last_valid_yaw'):
                return read_yaw.last_valid_yaw
    except Exception as e:
        logger.error(f"Error processing ESP32 BNO055 yaw: {e}")
        if hasattr(read_yaw, 'last_valid_yaw'):
            return read_yaw.last_valid_yaw
    return 0.0
def force_imu_reset():
    """Force reset IMU reading state when stuck"""
    if hasattr(read_yaw, 'last_valid_yaw'):
        print(f"IMU RESET: Clearing stuck state (was: {read_yaw.last_valid_yaw:.1f} deg)")
        delattr(read_yaw, 'last_valid_yaw')
    if hasattr(read_yaw, 'stuck_count'):
        print(f"IMU RESET: Clearing stuck count (was: {read_yaw.stuck_count})")
        delattr(read_yaw, 'stuck_count')
    if hasattr(read_yaw, 'reading_history'):
        print(f"IMU RESET: Clearing reading history")
        delattr(read_yaw, 'reading_history')
    logger.info("IMU reading state forcibly reset")
def get_imu_debug_info():
    """Get debug information about IMU state"""
    info = {}
    if hasattr(read_yaw, 'last_valid_yaw'):
        info['last_valid'] = read_yaw.last_valid_yaw
    if hasattr(read_yaw, 'stuck_count'):
        info['stuck_count'] = read_yaw.stuck_count
    if hasattr(read_yaw, 'reading_history'):
        info['history'] = read_yaw.reading_history[-3:]
    return info
def read_from_esp32():
    """Legacy function - now uses improved sensor data retrieval"""
    return get_fresh_sensor_data()
def send_to_esp32(speed, steering):
    """Legacy function - now uses improved command sending"""
    send_to_esp32_improved(speed, steering)
def update_obstacle_memory(red_blocks, green_blocks):
    """Update obstacle memory and return enhanced block lists"""
    global last_seen_obstacles
    current_time = time.time()
    if red_blocks or green_blocks:
        last_seen_obstacles["red"] = red_blocks
        last_seen_obstacles["green"] = green_blocks
        last_seen_obstacles["timestamp"] = current_time
        logger.debug(f"Obstacle memory updated: {len(red_blocks)} red, {len(green_blocks)} green")
    else:
        time_since_last_detection = current_time - last_seen_obstacles["timestamp"]
        if time_since_last_detection < OBSTACLE_MEMORY_TIMEOUT:
            red_blocks = last_seen_obstacles["red"]
            green_blocks = last_seen_obstacles["green"]
            logger.debug(f"Using remembered obstacles: {len(red_blocks)} red, {len(green_blocks)} green (age: {time_since_last_detection:.2f}s)")
        else:
            last_seen_obstacles = {"red": [], "green": [], "timestamp": 0}
    return red_blocks, green_blocks
def detect_obstacles(frame):
    """Detect red and green obstacles and return their positions"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_blocks = []
    green_blocks = []
    min_area = 500
    for contour in red_contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(contour)
            if y + h < FRAME_H * 0.3:
                continue
            if y + h >= FRAME_H * 0.95:
                continue
            if h > 100 and w > 100:
                red_blocks.append((x, y, w, h))
                logger.debug(f"RED block detected: x={x}, y={y}, w={w}, h={h}, area={area}")
    for contour in green_contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(contour)
            if h > 100 and w > 100:
                green_blocks.append((x, y, w, h))
                logger.debug(f"GREEN block detected: x={x}, y={y}, w={w}, h={h}, area={area}")
    if red_blocks or green_blocks:
        logger.debug(f"Detection summary: {len(red_blocks)} red blocks, {len(green_blocks)} green blocks")
    return red_blocks, green_blocks
def detect_parking_boundaries(frame):
    """Detect the two magenta parking-lot limitations separately from traffic signs."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_magenta, upper_magenta)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boundaries = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 300:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if y + h < FRAME_H * 0.30:
            continue
        if w < 12 or h < 12:
            continue
        boundaries.append((x, y, w, h))
    boundaries.sort(key=lambda b: b[2] * b[3], reverse=True)
    return boundaries
def set_parking_phase(new_phase):
    """Move to a new timed parking phase."""
    global parking_phase, parking_phase_start
    parking_phase = new_phase
    parking_phase_start = time.time()
    logger.info(f"Parking phase -> {new_phase}")
    print(f"PARKING PHASE: {new_phase}")
def initialize_parking_navigation():
    """Record the starting heading and determine which side contains the outer wall."""
    global navigation_mode, parking_side, parking_start_heading, parking_final_heading
    global straight_heading_reference, parking_verified
    f_dist, l_dist, r_dist, _, _ = get_fresh_sensor_data()
    parking_start_heading = read_yaw()
    parking_final_heading = parking_start_heading
    straight_heading_reference = parking_start_heading
    parking_verified = False
    if l_dist == 999 and r_dist == 999:
        parking_side = "LEFT"
        logger.warning("Both side sensors were invalid at start; defaulting parking side to LEFT")
    else:
        parking_side = "LEFT" if l_dist < r_dist else "RIGHT"
    navigation_mode = "EXIT_PARKING"
    set_parking_phase("EXIT_TURN_AWAY")
    print(f"PARKING START: side={parking_side}, heading={parking_start_heading:.1f}, "
          f"distances(F:{f_dist}, L:{l_dist}, R:{r_dist})")
    logger.info(f"Parking start recorded: side={parking_side}, heading={parking_start_heading:.1f}, "
                f"F={f_dist}, L={l_dist}, R={r_dist}")
def get_parking_wall_distance(left_d, right_d):
    """Return the ultrasonic distance on the saved parking/outer-wall side."""
    return left_d if parking_side == "LEFT" else right_d
def calculate_parking_approach_steering(current_yaw, left_d, right_d):
    """Stay parallel while approaching and passing the parking bay."""
    heading_error = parking_final_heading - current_yaw
    correction = heading_error * 0.8
    wall_distance = get_parking_wall_distance(left_d, right_d)
    if wall_distance < 999:
        wall_error = wall_distance - PARKING_APPROACH_SIDE_DISTANCE
        wall_correction = min(12.0, abs(wall_error) * 0.6)
        if parking_side == "LEFT":
            correction += -wall_correction if wall_error > 0 else wall_correction
        else:
            correction += wall_correction if wall_error > 0 else -wall_correction
    return min(max(SERVO_LEFT, SERVO_STRAIGHT + correction), SERVO_RIGHT)
def update_parking_controller(current_yaw, front_d, left_d, right_d, parking_blocks):
    """Return speed, steering and state for parking-specific navigation modes."""
    global navigation_mode, parking_detection_streak, parking_verified
    now = time.time()
    phase_elapsed = now - parking_phase_start
    if navigation_mode == "PARKED":
        state = "PARKED" if parking_verified else "PARKED_CHECK_REQUIRED"
        return STOP_SPEED, SERVO_STRAIGHT, state
    if navigation_mode == "EXIT_PARKING":
        if parking_phase == "EXIT_TURN_AWAY":
            target = parking_start_heading + (PARKING_EXIT_ANGLE if parking_side == "LEFT" else -PARKING_EXIT_ANGLE)
            steering = SERVO_RIGHT if parking_side == "LEFT" else SERVO_LEFT
            if abs(target - current_yaw) <= PARKING_HEADING_TOLERANCE or phase_elapsed > 2.5:
                set_parking_phase("EXIT_COUNTER_STEER")
            return PARKING_SPEED, steering, "EXIT_PARKING_TURN_AWAY"
        if parking_phase == "EXIT_COUNTER_STEER":
            steering = SERVO_LEFT if parking_side == "LEFT" else SERVO_RIGHT
            if abs(parking_start_heading - current_yaw) <= PARKING_HEADING_TOLERANCE:
                set_parking_phase("EXIT_CLEAR")
            elif phase_elapsed > 3.0:
                logger.warning("Parking exit counter-steer timed out; continuing cautiously")
                set_parking_phase("EXIT_CLEAR")
            return PARKING_SPEED, steering, "EXIT_PARKING_COUNTER_STEER"
        if parking_phase == "EXIT_CLEAR":
            steering = min(max(
                SERVO_LEFT,
                SERVO_STRAIGHT + (parking_start_heading - current_yaw) * 0.8
            ), SERVO_RIGHT)
            if phase_elapsed >= 0.65:
                navigation_mode = "NAVIGATING"
                set_parking_phase("NONE")
                logger.info("Parking exit complete; starting three-lap navigation")
                print("PARKING EXIT COMPLETE: Starting three laps")
            return PARKING_SPEED, steering, "EXIT_PARKING_CLEAR"
    if navigation_mode == "FINAL_APPROACH":
        close_boundary = False
        if parking_blocks:
            x, y, w, h = parking_blocks[0]
            close_boundary = (w * h >= PARKING_CLOSE_BLOCK_AREA or
                              y + h >= PARKING_CLOSE_BLOCK_BOTTOM)
        parking_detection_streak = parking_detection_streak + 1 if close_boundary else 0
        if parking_detection_streak >= 3:
            navigation_mode = "PARKING"
            set_parking_phase("PASS_FRONT_LIMIT")
            return PARKING_SPEED, calculate_parking_approach_steering(
                current_yaw, left_d, right_d
            ), "PARKING_BOUNDARY_CONFIRMED"
        if front_d < 22:
            return STOP_SPEED, SERVO_STRAIGHT, "FINAL_APPROACH_BLOCKED"
        return PARKING_SPEED, calculate_parking_approach_steering(
            current_yaw, left_d, right_d
        ), "FINAL_APPROACH_SEARCHING_MAGENTA"
    if navigation_mode == "PARKING":
        if parking_phase == "PASS_FRONT_LIMIT":
            steering = calculate_parking_approach_steering(current_yaw, left_d, right_d)
            if phase_elapsed >= PARKING_PASS_BLOCK_DURATION:
                set_parking_phase("REVERSE_TOWARD_BAY")
            return PARKING_SPEED, steering, "PARKING_PASS_FRONT_LIMIT"
        if parking_phase == "REVERSE_TOWARD_BAY":
            target = parking_final_heading + (PARKING_REVERSE_ANGLE if parking_side == "LEFT" else -PARKING_REVERSE_ANGLE)
            steering = SERVO_LEFT if parking_side == "LEFT" else SERVO_RIGHT
            if abs(target - current_yaw) <= PARKING_HEADING_TOLERANCE:
                set_parking_phase("REVERSE_COUNTER_STEER")
            elif phase_elapsed > PARKING_PHASE_TIMEOUT:
                logger.error("Reverse-toward-bay phase timed out; stopping for safety")
                navigation_mode = "PARKED"
                parking_verified = False
                return STOP_SPEED, SERVO_STRAIGHT, "PARKING_TIMEOUT_STOP"
            return -PARKING_REVERSE_SPEED, steering, "PARKING_REVERSE_TOWARD_BAY"
        if parking_phase == "REVERSE_COUNTER_STEER":
            steering = SERVO_RIGHT if parking_side == "LEFT" else SERVO_LEFT
            if abs(parking_final_heading - current_yaw) <= PARKING_HEADING_TOLERANCE:
                set_parking_phase("FINAL_REVERSE_STRAIGHT")
            elif phase_elapsed > PARKING_PHASE_TIMEOUT:
                logger.error("Reverse counter-steer phase timed out; stopping for safety")
                navigation_mode = "PARKED"
                parking_verified = False
                return STOP_SPEED, SERVO_STRAIGHT, "PARKING_TIMEOUT_STOP"
            return -PARKING_REVERSE_SPEED, steering, "PARKING_REVERSE_COUNTER_STEER"
        if parking_phase == "FINAL_REVERSE_STRAIGHT":
            if phase_elapsed >= PARKING_FINAL_REVERSE_DURATION:
                set_parking_phase("VERIFY")
                return STOP_SPEED, SERVO_STRAIGHT, "PARKING_STOP_FOR_VERIFY"
            steering = min(max(
                SERVO_LEFT,
                SERVO_STRAIGHT + (parking_final_heading - current_yaw) * 0.8
            ), SERVO_RIGHT)
            return -PARKING_REVERSE_SPEED, steering, "PARKING_FINAL_REVERSE"
        if parking_phase == "VERIFY":
            heading_ok = abs(parking_final_heading - current_yaw) <= PARKING_HEADING_TOLERANCE
            wall_distance = get_parking_wall_distance(left_d, right_d)
            wall_ok = wall_distance < PARKING_INSIDE_SIDE_DISTANCE
            parking_verified = heading_ok and wall_ok and bool(parking_blocks)
            navigation_mode = "PARKED"
            set_parking_phase("COMPLETE")
            print(f"PARKING COMPLETE: verified={parking_verified}, heading_ok={heading_ok}, "
                  f"wall_distance={wall_distance}, magenta_visible={bool(parking_blocks)}")
            logger.info(f"Parking complete: verified={parking_verified}, heading_ok={heading_ok}, "
                        f"wall_distance={wall_distance}, magenta_visible={bool(parking_blocks)}")
            return STOP_SPEED, SERVO_STRAIGHT, "PARKED" if parking_verified else "PARKED_CHECK_REQUIRED"
    return STOP_SPEED, SERVO_STRAIGHT, "PARKING_FAILSAFE_STOP"
def get_region(x):
    """Determine which region the x-coordinate falls into"""
    if LEFT_REGION[0] <= x < LEFT_REGION[1]:
        return "LEFT"
    elif CENTER_REGION[0] <= x < CENTER_REGION[1]:
        return "CENTER"
    elif RIGHT_REGION[0] <= x < RIGHT_REGION[1]:
        return "RIGHT"
    return "UNKNOWN"
def calculate_turn_intensity(block_area):
    """Calculate turn intensity based on block size (area)"""
    if block_area <= SIZE_THRESHOLD_SMALL:
        intensity = MIN_TURN
        size_category = "SMALL"
    elif block_area >= SIZE_THRESHOLD_LARGE:
        intensity = MAX_TURN
        size_category = "LARGE"
    else:
        ratio = (block_area - SIZE_THRESHOLD_SMALL) / (SIZE_THRESHOLD_LARGE - SIZE_THRESHOLD_SMALL)
        intensity = MIN_TURN + (MAX_TURN - MIN_TURN) * ratio
        size_category = "MEDIUM"
    print(f"Block area: {block_area} -> {size_category} -> Turn intensity: {intensity:.1f}")
    logger.debug(f"Turn calculation: area={block_area} -> {size_category} -> intensity={intensity:.1f}")
    return intensity, size_category
def calculate_steering(blocks, color):
    """Calculate steering based on block positions, size, and WRO rules with edge avoidance optimization"""
    if not blocks:
        return 0, "NO_OBSTACLE"
    largest_block = max(blocks, key=lambda b: b[2] * b[3])
    x, y, w, h = largest_block
    block_center_x = x + w // 2
    block_area = w * h
    region = get_region(block_center_x)
    edge_threshold = 100
    if color == "RED" and x < edge_threshold:
        print(f"RED block at LEFTMOST EDGE (x={x}) - Going STRAIGHT (natural avoidance)")
        logger.info(f"RED block at extreme left edge (x={x}) - no steering adjustment needed")
        return 0, "STRAIGHT_EDGE_AVOIDANCE"
    if color == "GREEN" and (x + w) > (FRAME_W - edge_threshold):
        print(f"GREEN block at RIGHTMOST EDGE (x+w={x+w}) - Going STRAIGHT (natural avoidance)")
        logger.info(f"GREEN block at extreme right edge (x+w={x+w}) - no steering adjustment needed")
        return 0, "STRAIGHT_EDGE_AVOIDANCE"
    base_intensity, size_category = calculate_turn_intensity(block_area)
    steering_adjustment = 0
    turn_type = ""
    if color == "GREEN":
        if region == "LEFT":
            steering_adjustment = -base_intensity
            turn_type = f"{size_category}_LEFT"
        elif region == "CENTER":
            steering_adjustment = -(base_intensity * 1.2)
            turn_type = f"{size_category}_CENTER_LEFT"
        elif region == "RIGHT":
            steering_adjustment = -(base_intensity * 0.8)
            turn_type = f"{size_category}_SLIGHT_LEFT"
    elif color == "RED":
        if region == "LEFT":
            steering_adjustment = base_intensity * 0.8
            turn_type = f"{size_category}_SLIGHT_RIGHT"
        elif region == "CENTER":
            steering_adjustment = base_intensity * 1.2
            turn_type = f"{size_category}_CENTER_RIGHT"
        elif region == "RIGHT":
            steering_adjustment = base_intensity
            turn_type = f"{size_category}_RIGHT"
    max_adjustment = 55
    steering_adjustment = max(-max_adjustment, min(max_adjustment, steering_adjustment))
    print(f"{color} block in {region} region at ({block_center_x}, {y}) -> {turn_type} (area={block_area})")
    logger.debug(f"Steering calculation: {color} block in {region} at ({block_center_x},{y}) -> {turn_type} | area={block_area} | adjustment={steering_adjustment:.1f}")
    return steering_adjustment, turn_type
def decide_steering():
    """Main decision logic for steering with improved sensor data handling"""
    global yaw, last_turn_direction, recovery_mode, obstacle_count, turn_phase
    global turn_just_completed, obstacle_deflection_after_turn
    yaw = read_yaw()
    f_dist, l_dist, r_dist, data_fresh, data_age = get_fresh_sensor_data()
    if not data_fresh and data_age > 1.0:
        logger.warning(f"Sensor data is stale (age: {data_age:.2f}s)")
    frame = picam2.capture_array()
    red_blocks, green_blocks = detect_obstacles(frame)
    parking_blocks = detect_parking_boundaries(frame)
    red_blocks, green_blocks = update_obstacle_memory(red_blocks, green_blocks)
    steering = SERVO_STRAIGHT
    state = "STRAIGHT"
    imu_data_age = time.time() - last_imu_update if last_imu_update > 0 else 999
    if data_age > 1.0 or imu_data_age > 1.0:
        state = "SENSOR_DATA_STALE_STOP"
        send_to_esp32_improved(STOP_SPEED, SERVO_STRAIGHT)
        logger.error(
            f"Navigation stopped for stale ESP data: packet_age={data_age:.2f}s, "
            f"imu_age={imu_data_age:.2f}s"
        )
        frame_debug = draw_debug(
            frame, red_blocks, green_blocks, SERVO_STRAIGHT, state, parking_blocks
        )
        cv2.imshow("Region-Based Navigation", frame_debug)
        return SERVO_STRAIGHT, state
    if in_turn_mode:
        turn_result = update_turn_progress()
        if turn_result is not None and len(turn_result) == 2:
            turn_speed, turn_steering = turn_result
            steering = turn_steering
            state = f"TURNING_{turn_phase}"
            send_to_esp32_improved(turn_speed, steering)
            log_navigation_data(steering, state, yaw, f_dist, l_dist, r_dist, red_blocks, green_blocks)
            frame_debug = draw_debug(frame, red_blocks, green_blocks, steering, state, parking_blocks)
            cv2.imshow("Region-Based Navigation", frame_debug)
            return steering, state
        else:
            steering = SERVO_STRAIGHT
            state = "TURN_COMPLETED"
    parking_control_active = navigation_mode in ("EXIT_PARKING", "PARKING", "PARKED")
    final_approach_clear = (navigation_mode == "FINAL_APPROACH" and
                            len(red_blocks) == 0 and len(green_blocks) == 0)
    if parking_control_active or final_approach_clear:
        if data_age > 1.0:
            parking_speed, steering, state = STOP_SPEED, SERVO_STRAIGHT, "PARKING_SENSOR_STALE_STOP"
            logger.error(f"Parking motion blocked because sensor data is stale ({data_age:.2f}s)")
        else:
            parking_speed, steering, state = update_parking_controller(
                yaw, f_dist, l_dist, r_dist, parking_blocks
            )
        send_to_esp32_improved(parking_speed, steering)
        log_navigation_data(steering, state, yaw, f_dist, l_dist, r_dist, red_blocks, green_blocks)
        frame_debug = draw_debug(frame, red_blocks, green_blocks, steering, state, parking_blocks)
        cv2.imshow("Region-Based Navigation", frame_debug)
        return steering, state
    if navigation_mode == "NAVIGATING" and len(red_blocks) == 0 and len(green_blocks) == 0:
        turn_direction = should_initiate_turn(f_dist, l_dist, r_dist, red_blocks, green_blocks)
        if turn_direction is not None:
            if execute_turn(turn_direction):
                turn_result = update_turn_progress()
                if turn_result is not None and len(turn_result) == 2:
                    turn_speed, turn_steering = turn_result
                    steering = turn_steering
                    state = f"INITIATING_{turn_direction}_TURN"
                    send_to_esp32_improved(turn_speed, steering)
                    log_navigation_data(steering, state, yaw, f_dist, l_dist, r_dist, red_blocks, green_blocks)
                    frame_debug = draw_debug(frame, red_blocks, green_blocks, steering, state, parking_blocks)
                    cv2.imshow("Region-Based Navigation", frame_debug)
                    return steering, state
    else:
        logger.debug(f"Obstacle avoidance active - turn logic disabled (R:{len(red_blocks)}, G:{len(green_blocks)})")
    closest_block = None
    closest_color = None
    closest_area = 0
    if red_blocks or green_blocks:
        all_red_areas = [w * h for x, y, w, h in red_blocks]
        all_green_areas = [w * h for x, y, w, h in green_blocks]
        logger.debug(f"All detections: RED blocks={len(red_blocks)}{all_red_areas}, GREEN blocks={len(green_blocks)}{all_green_areas}")
    if red_blocks:
        largest_red = max(red_blocks, key=lambda b: b[2] * b[3])
        red_area = largest_red[2] * largest_red[3]
        if red_area > closest_area:
            closest_block = largest_red
            closest_color = "RED"
            closest_area = red_area
            logger.debug(f"Largest RED block: area={red_area}")
    if green_blocks:
        largest_green = max(green_blocks, key=lambda b: b[2] * b[3])
        green_area = largest_green[2] * largest_green[3]
        if green_area > closest_area:
            closest_block = largest_green
            closest_color = "GREEN"
            closest_area = green_area
            logger.debug(f"Largest GREEN block: area={green_area}")
        elif green_blocks:
            logger.debug(f"GREEN block present but smaller: area={green_area} vs RED area={closest_area}")
    if closest_block is not None:
        if in_turn_mode:
            print("WARNING: Canceling turn due to obstacle detection!")
            logger.warning("Turn canceled - obstacle detected during turn execution")
            complete_turn(count_as_completed=False)
        if turn_just_completed and not obstacle_deflection_after_turn:
            obstacle_deflection_after_turn = True
            print("SEQUENCE CONTROL: Obstacle deflection detected after turn - Next turn now allowed!")
            logger.info("Obstacle deflection after turn detected - turn sequence constraint lifted")
        closest_blocks = [closest_block]
        adjustment, turn_type = calculate_steering(closest_blocks, closest_color)
        steering = SERVO_STRAIGHT + adjustment
        state = f"{closest_color}_{turn_type}"
        if turn_type == "STRAIGHT_EDGE_AVOIDANCE":
            logger.info(f"Edge avoidance maneuver - counts as active obstacle deflection")
        elif abs(adjustment) < 20:
            logger.debug(f"Minor obstacle adjustment ({adjustment:.1f}) - not counted as active deflection")
        else:
            logger.debug(f"Significant obstacle adjustment ({adjustment:.1f}) - counts as active deflection")
        if closest_color == "GREEN":
            last_turn_direction = "LEFT"
        else:
            last_turn_direction = "RIGHT"
        recovery_mode = False
        global post_obstacle_delay_active, post_obstacle_delay_start
        if abs(adjustment) > 10:
            post_obstacle_delay_active = True
            post_obstacle_delay_start = time.time()
            print(f"POST-OBSTACLE DELAY: Started {POST_OBSTACLE_DELAY_DURATION}s delay after deflecting {closest_color} obstacle")
            logger.debug(f"Post-obstacle delay activated: {POST_OBSTACLE_DELAY_DURATION}s delay after {closest_color} deflection")
        if abs(adjustment) > 20:
            current_obstacle_key = f"{closest_color}_{closest_area//1000}"
            if not hasattr(decide_steering, 'last_obstacle_key'):
                decide_steering.last_obstacle_key = ""
                decide_steering.obstacle_stable_count = 0
            if current_obstacle_key != decide_steering.last_obstacle_key:
                decide_steering.obstacle_stable_count = 1
                decide_steering.last_obstacle_key = current_obstacle_key
                logger.debug(f"New obstacle detected: {current_obstacle_key}")
            else:
                decide_steering.obstacle_stable_count += 1
                if decide_steering.obstacle_stable_count == 5:
                    obstacle_count += 1
                    print(f"OBSTACLE #{obstacle_count} confirmed: {closest_color} block (area={closest_area})")
                    logger.info(f"Obstacle #{obstacle_count} confirmed and being avoided: {closest_color} block (area={closest_area})")
        logger.info(f"PRIORITY: {closest_color} block (area={closest_area}) is closest - handling first")
        print(f"AVOIDING: {closest_color} block (area={closest_area}) - Total obstacles: {obstacle_count}/{max_obstacles_per_segment}")
    else:
        logger.debug(f"No obstacles detected - red_blocks={len(red_blocks)}, green_blocks={len(green_blocks)}")
        if hasattr(decide_steering, 'last_obstacle_key'):
            decide_steering.last_obstacle_key = ""
            decide_steering.obstacle_stable_count = 0
        if post_obstacle_delay_active:
            delay_elapsed = time.time() - post_obstacle_delay_start
            if delay_elapsed < POST_OBSTACLE_DELAY_DURATION:
                if last_turn_direction == "RIGHT":
                    recovery_steering = 10
                    state = "POST_OBSTACLE_DELAY_RIGHT"
                elif last_turn_direction == "LEFT":
                    recovery_steering = -10
                    state = "POST_OBSTACLE_DELAY_LEFT"
                else:
                    recovery_steering = 0
                    state = "POST_OBSTACLE_DELAY"
                error = straight_heading_reference - yaw
                yaw_correction = error * 0.3
                total_correction = recovery_steering + yaw_correction
                steering = SERVO_STRAIGHT + total_correction
                remaining_delay = POST_OBSTACLE_DELAY_DURATION - delay_elapsed
                print(f"POST-OBSTACLE DELAY: {remaining_delay:.1f}s remaining, maintaining deflection")
                logger.debug(f"Post-obstacle delay active: {remaining_delay:.1f}s remaining")
                steering = min(max(SERVO_LEFT, steering), SERVO_RIGHT)
                return steering, state
            else:
                post_obstacle_delay_active = False
                print(f"POST-OBSTACLE DELAY: Complete after {delay_elapsed:.1f}s - resuming normal navigation")
                logger.debug(f"Post-obstacle delay completed after {delay_elapsed:.1f}s")
        recovery_steering = 0
        heading_error = yaw - straight_heading_reference
        if abs(heading_error) > 5:
            if last_turn_direction == "RIGHT" and heading_error > 5:
                recovery_steering = -15
                state = "RECOVERY_LEFT_AFTER_RIGHT"
                recovery_mode = True
            elif last_turn_direction == "LEFT" and heading_error < -5:
                recovery_steering = 15
                state = "RECOVERY_RIGHT_AFTER_LEFT"
                recovery_mode = True
        error = straight_heading_reference - yaw
        yaw_correction = error * 0.8
        total_correction = recovery_steering + yaw_correction
        steering = SERVO_STRAIGHT + total_correction
        if not recovery_mode:
            state = "STRAIGHT_YAW_CORRECTION"
        print(f"No obstacles - Yaw: {yaw:.1f}, Target: {straight_heading_reference:.1f}, Recovery: {recovery_steering}, YawCorr: {yaw_correction:.1f}, Total: {total_correction:.1f}")
        logger.debug(f"Recovery mode: yaw={yaw:.1f}, target={straight_heading_reference:.1f}, recovery_steering={recovery_steering}, yaw_correction={yaw_correction:.1f}, total={total_correction:.1f}")
        heading_error = abs(yaw - straight_heading_reference)
        if heading_error < 3:
            last_turn_direction = ""
            recovery_mode = False
    steering = min(max(SERVO_LEFT, steering), SERVO_RIGHT)
    log_navigation_data(steering, state, yaw, f_dist, l_dist, r_dist, red_blocks, green_blocks)
    frame_debug = draw_debug(frame, red_blocks, green_blocks, steering, state, parking_blocks)
    cv2.imshow("Region-Based Navigation", frame_debug)
    return steering, state
def draw_debug(frame, red_blocks, green_blocks, steering, state, parking_blocks=None):
    """Draw debug visualization with regions and obstacles"""
    global video_writer, turn_just_completed, obstacle_deflection_after_turn
    debug_frame = frame.copy()
    parking_blocks = parking_blocks or []
    cv2.line(debug_frame, (FRAME_W // 3, 0), (FRAME_W // 3, FRAME_H), (255, 255, 255), 2)
    cv2.line(debug_frame, (2 * FRAME_W // 3, 0), (2 * FRAME_W // 3, FRAME_H), (255, 255, 255), 2)
    edge_threshold = 100
    cv2.line(debug_frame, (edge_threshold, 0), (edge_threshold, FRAME_H), (255, 0, 255), 1)
    cv2.line(debug_frame, (FRAME_W - edge_threshold, 0), (FRAME_W - edge_threshold, FRAME_H), (255, 0, 255), 1)
    cv2.putText(debug_frame, "LEFT", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(debug_frame, "CENTER", (FRAME_W // 2 - 50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(debug_frame, "RIGHT", (FRAME_W - 150, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(debug_frame, "EDGE", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    cv2.putText(debug_frame, "EDGE", (FRAME_W - 60, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    edge_threshold = 100
    for x, y, w, h in red_blocks:
        in_edge_zone = x < edge_threshold
        border_color = (255, 0, 255) if in_edge_zone else (0, 0, 255)
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), border_color, 3)
        center_x = x + w // 2
        area = w * h
        region = get_region(center_x)
        edge_label = "-EDGE" if in_edge_zone else ""
        cv2.putText(debug_frame, f"RED-{region}{edge_label}", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, border_color, 2)
        cv2.putText(debug_frame, f"Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, border_color, 2)
    for x, y, w, h in green_blocks:
        in_edge_zone = (x + w) > (FRAME_W - edge_threshold)
        border_color = (255, 0, 255) if in_edge_zone else (0, 255, 0)
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), border_color, 3)
        center_x = x + w // 2
        area = w * h
        region = get_region(center_x)
        edge_label = "-EDGE" if in_edge_zone else ""
        cv2.putText(debug_frame, f"GREEN-{region}{edge_label}", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, border_color, 2)
        cv2.putText(debug_frame, f"Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, border_color, 2)
    for x, y, w, h in parking_blocks:
        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (255, 0, 255), 4)
        cv2.putText(debug_frame, f"PARKING {w * h}", (x, max(20, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    cv2.putText(debug_frame, timestamp_str, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    queue_size = len(command_queue) if command_queue else 0
    data_age = time.time() - last_sensor_update if last_sensor_update > 0 else 999
    cv2.putText(debug_frame, f"Steering: {steering:.1f}", (10, FRAME_H - 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    state_color = (0, 255, 255) if "STRAIGHT_EDGE_AVOIDANCE" in state else (255, 255, 0)
    cv2.putText(debug_frame, f"State: {state}", (10, FRAME_H - 210), cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2)
    if "STRAIGHT_EDGE_AVOIDANCE" in state:
        cv2.putText(debug_frame, "EDGE AVOID!", (300, FRAME_H - 210), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 3)
    cv2.putText(debug_frame, f"Yaw: {yaw:.1f}", (10, FRAME_H - 180), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Target: {straight_heading_reference:.1f}", (10, FRAME_H - 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Obstacles: {obstacle_count}/{max_obstacles_per_segment}", (10, FRAME_H - 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    turns_color = (0, 0, 255) if total_turns_completed >= MAX_TURNS_BEFORE_STOP - 2 else (255, 255, 0)
    cv2.putText(debug_frame, f"Turns: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP}", (300, FRAME_H - 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, turns_color, 2)
    if in_turn_mode:
        cv2.putText(debug_frame, f"{turn_type}: {turn_phase} -> Target: {target_yaw:.1f}", (10, FRAME_H - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    else:
        cv2.putText(debug_frame, f"Last Turn: {last_turn_direction}", (10, FRAME_H - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    if turn_just_completed and not obstacle_deflection_after_turn:
        sequence_status = "BLOCKED"
        sequence_color = (0, 0, 255)
        sequence_detail = f"(Need deflection)"
    elif turn_just_completed and obstacle_deflection_after_turn:
        sequence_status = "ALLOWED"
        sequence_color = (0, 255, 0)
        sequence_detail = f"(Post-deflection)"
    else:
        sequence_status = "BLOCKED"
        sequence_color = (0, 255, 255)
        sequence_detail = f"(Initial state)"
    cv2.putText(debug_frame, f"Next Turn: {sequence_status}", (300, FRAME_H - 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, sequence_color, 2)
    cv2.putText(debug_frame, sequence_detail, (300, FRAME_H - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, sequence_color, 2)
    cv2.putText(debug_frame, f"Recovery: {recovery_mode}", (10, FRAME_H - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Cmd Queue: {queue_size}", (10, FRAME_H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Data Age: {data_age:.2f}s", (300, FRAME_H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Mode: {navigation_mode}", (FRAME_W - 430, FRAME_H - 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 255), 2)
    cv2.putText(debug_frame, f"Parking: {parking_side}/{parking_phase}",
                (FRAME_W - 430, FRAME_H - 60), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (255, 0, 255), 2)
    cv2.putText(
        debug_frame,
        f"BNO Cal: {imu_system_calibration}/{imu_gyro_calibration}/"
        f"{imu_accel_calibration}/{imu_mag_calibration}",
        (FRAME_W - 430, FRAME_H - 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2
    )
    if in_turn_mode:
        if turn_type == "ARC":
            phase_color = (0, 255, 0)
            display_text = f">> {turn_type} <<"
        else:
            phase_color = (255, 255, 0) if turn_phase == "FORWARD" else (0, 255, 255) if turn_phase == "REVERSE_TURN" else (0, 255, 0)
            display_text = f">> {turn_type}: {turn_phase} <<"
        cv2.putText(debug_frame, display_text, (FRAME_W - 350, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, phase_color, 3)
    if RECORD_VIDEO and video_writer is not None:
        bgr_frame = cv2.cvtColor(debug_frame, cv2.COLOR_RGB2BGR)
        video_writer.write(debug_frame)
    return debug_frame
def main():
    """Main navigation loop with improved serial communication"""
    global yaw, video_writer
    try:
        picam2.configure(config)
        picam2.start()
        if RECORD_VIDEO:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            video_writer = cv2.VideoWriter(VIDEO_FILE, fourcc, 20.0, (FRAME_W, FRAME_H))
            logger.info(f"Video recording initialized: {VIDEO_FILE}")
        print("Testing serial connection to ESP32...")
        try:
            ser.reset_output_buffer()
            print(f"Serial port: {SERIAL_PORT}")
            print(f"Baud rate: {BAUD_RATE}")
            print(f"Port open: {ser.is_open}")
            logger.info(f"Serial connection test: Port={SERIAL_PORT}, Baud={BAUD_RATE}, Open={ser.is_open}")
            ser.write(b"0,90\n")
            ser.flush()
            print("Robot stopped; waiting for ESP32 START signal")
            logger.info("Initial stop command sent; waiting for ESP32 START signal")
        except Exception as e:
            print(f"ERROR: Serial connection issue: {e}")
            logger.error(f"Serial connection test failed: {e}")
        serial_thread = threading.Thread(target=serial_communication_handler, daemon=True)
        serial_thread.start()
        logger.info("Serial communication thread started")
        time.sleep(0.5)
        logger.info("System initialized successfully")
        logger.info(f"Log file: {LOG_FILE}")
        logger.info(f"Video file: {VIDEO_FILE}")
        logger.info("Waiting for START signal from ESP32...")
        print("=== IMPROVED RPi NAVIGATION SYSTEM ===")
        print("Features:")
        print("- Non-blocking serial communication")
        print("- Camera is the only sensor connected directly to the Raspberry Pi")
        print("- ESP32 supplies all ultrasonic and BNO055 data")
        print("- Command queuing to prevent pile-up")
        print("- Fresh sensor data monitoring")
        print("- ESP-pushed sensor packets and rate-limited 20Hz motor commands")
        print("Waiting for ESP32 to send START...")
        while not esp_start_event.wait(timeout=0.1):
            if not serial_thread.is_alive():
                logger.error("Serial communication thread died while waiting for START! Restarting...")
                serial_thread = threading.Thread(target=serial_communication_handler, daemon=True)
                serial_thread.start()
        print("START received from ESP32! Starting system...")
        logger.info("START received from ESP32! Starting navigation system...")
        calibrate_imu()
        logger.info(f"IMU calibrated with initial heading: {initial_yaw:.1f}")
        global turn_just_completed, obstacle_deflection_after_turn
        turn_just_completed = True
        obstacle_deflection_after_turn = False
        print("SEQUENCE CONTROL: Initialized - Turning BLOCKED by default until obstacle deflection occurs")
        logger.info("Turn sequence control initialized - turns BLOCKED by default, requires obstacle deflection to enable")
        time.sleep(0.2)
        initialize_parking_navigation()
        print("Starting region-based navigation with improved communication...")
        print(f"Regions: LEFT(0-{LEFT_REGION[1]}), CENTER({CENTER_REGION[0]}-{CENTER_REGION[1]}), RIGHT({RIGHT_REGION[0]}-{FRAME_W})")
        logger.info("Starting region-based navigation...")
        logger.info(f"Frame regions: LEFT(0-{LEFT_REGION[1]}), CENTER({CENTER_REGION[0]}-{CENTER_REGION[1]}), RIGHT({RIGHT_REGION[0]}-{FRAME_W})")
        logger.info(f"Turn intensity config: MIN={MIN_TURN}, MAX={MAX_TURN}, SMALL_THRESHOLD={SIZE_THRESHOLD_SMALL}, LARGE_THRESHOLD={SIZE_THRESHOLD_LARGE}")
        logger.info(f"Communication config: CMD_INTERVAL={COMMAND_INTERVAL}s, Queue_size={command_queue.maxlen}")
        loop_count = 0
        start_time = time.time()
        last_status_time = 0
        while True:
            loop_count += 1
            loop_start_time = time.time()
            steering, state = decide_steering()
            internally_commanded_states = (
                "EXIT_PARKING", "FINAL_APPROACH", "PARKING", "PARKED", "SENSOR_DATA_STALE"
            )
            if not in_turn_mode and not state.startswith(internally_commanded_states):
                send_to_esp32_improved(BASE_SPEED, steering)
            current_time = time.time()
            if current_time - last_status_time > 1.0:
                queue_size = len(command_queue)
                data_age = current_time - last_sensor_update if last_sensor_update > 0 else 999
                loop_hz = loop_count / (current_time - start_time) if current_time > start_time else 0
                print(f"Loop {loop_count} ({loop_hz:.1f}Hz) | Yaw: {yaw:.1f} | Steering: {steering:.1f} | State: {state}")
                print(f"Distances - Front: {front_dist}, Left: {left_dist}, Right: {right_dist} (Age: {data_age:.2f}s)")
                imu_age = current_time - last_imu_update if last_imu_update > 0 else 999
                print(f"ESP BNO055 - Raw: {imu_raw_yaw} | Age: {imu_age:.2f}s | Cal: "
                      f"{imu_system_calibration}/{imu_gyro_calibration}/"
                      f"{imu_accel_calibration}/{imu_mag_calibration}")
                print(f"Turns Completed: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP} | Command Queue: {queue_size}/{command_queue.maxlen} | Serial Thread: {'Active' if serial_thread.is_alive() else 'DEAD'}")
                print(f"Navigation: {navigation_mode} | Parking side: {parking_side} | Phase: {parking_phase} | Verified: {parking_verified}")
                print("-" * 80)
                last_status_time = current_time
            if not serial_thread.is_alive():
                logger.error("Serial communication thread died! Restarting...")
                serial_thread = threading.Thread(target=serial_communication_handler, daemon=True)
                serial_thread.start()
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("User pressed 'q' - exiting navigation loop")
                break
            elif key == ord('t'):
                if in_turn_mode:
                    print("Manual turn escape activated!")
                    logger.info("Manual turn escape - aborting turn without counting corner")
                    complete_turn(count_as_completed=False)
            elif key == ord('r'):
                obstacle_count = 0
                print(f"Obstacle count reset to 0")
                logger.info("Manual obstacle count reset")
            loop_duration = time.time() - loop_start_time
            if loop_duration < 0.05:
                time.sleep(0.05 - loop_duration)
        end_time = time.time()
        total_runtime = end_time - start_time
        avg_hz = loop_count / total_runtime if total_runtime > 0 else 0
        logger.info(f"Navigation session completed:")
        logger.info(f"  Total loops: {loop_count}")
        logger.info(f"  Runtime: {total_runtime:.2f} seconds")
        logger.info(f"  Average frequency: {avg_hz:.1f} Hz")
        logger.info(f"  Turns completed: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP}")
        logger.info(f"  Final queue size: {len(command_queue)}")
        print(f"\n=== SESSION STATISTICS ===")
        print(f"Total loops: {loop_count}")
        print(f"Runtime: {total_runtime:.2f} seconds")
        print(f"Average frequency: {avg_hz:.1f} Hz")
        print(f"Turns completed: {total_turns_completed}/{MAX_TURNS_BEFORE_STOP}")
        print(f"Final queue size: {len(command_queue)}")
        if navigation_mode == "PARKED":
            print(f"MISSION COMPLETED: Three laps and parking finished (verified={parking_verified})")
            logger.info(f"Mission completed: three laps and parking finished, verified={parking_verified}")
    except KeyboardInterrupt:
        print("Interrupted by user")
        logger.info("Navigation interrupted by user (Ctrl+C)")
    except Exception as e:
        print(f"Error: {e}")
        logger.error(f"Navigation error: {e}", exc_info=True)
    finally:
        print("Cleaning up...")
        logger.info("Starting cleanup process...")
        try:
            with serial_lock:
                ser.write(b"0,90\n")
                ser.flush()
            logger.info("Emergency stop sent to ESP32")
        except Exception as e:
            logger.error(f"Failed to send emergency stop: {e}")
        command_queue.clear()
        logger.info("Command queue cleared")
        if 'picam2' in globals():
            picam2.stop()
            logger.info("Camera stopped")
        if video_writer is not None:
            video_writer.release()
            logger.info(f"Video recording saved: {VIDEO_FILE}")
        cv2.destroyAllWindows()
        logger.info("System shutdown complete")
        print("System shutdown complete")
if __name__ == "__main__":
    main()
