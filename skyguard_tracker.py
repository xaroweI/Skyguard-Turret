import cv2
import numpy as np
import time
import sys
import serial
import atexit
import traceback

# === CONFIGURATION ===
URL = "http://balblabla:8080/video"
CAM_WIDTH, CAM_HEIGHT = 320, 240
MIN_CONTOUR_AREA = 800
SMOOTHING_ALPHA = 0.3
PREDICTION_STEPS = 5
RECONNECT_DELAY = 2.0

PAN_MIN_ANGLE, PAN_MAX_ANGLE = 0, 180
TILT_MIN_ANGLE, TILT_MAX_ANGLE = 0, 180

SWEEP_MIN = 0
SWEEP_MAX = 180
SWEEP_INCREMENT = 1.0
SWEEP_TILT = 90

SEND_THRESHOLD = 2
LOCK_HOLD_FRAMES = 60       # frames to keep predicting after target lost (~2s at 30fps)
HOLD_TIMEOUT_FRAMES = 240   # frames to hold last position before giving up and sweeping again
WARMUP_FRAMES = 30          # frames to skip so background subtractor can initialise


SERIAL_PORT = 'COM5'
SERIAL_BAUD = 115200
SERIAL_RETRIES = 3
SERIAL_RECONNECT_DELAY = 2.0

# Calibration positions: (pan_angle, tilt_angle, axis, label)
# The turret moves to each angle; you click where it is pointing in the live view.
CAL_STEPS = [
    (45,  90,  'pan',  'PAN LEFT'),
    (135, 90,  'pan',  'PAN RIGHT'),
    (90,  45,  'tilt', 'TILT UP'),
    (90,  135, 'tilt', 'TILT DOWN'),
]

# === STATE ===
sweep_direction = -1
pan_sweep_angle = float(SWEEP_MAX)

ser = None
last_sent_pan = None
last_sent_tilt = None

# Calibration data — populated by run_calibration(), used by map_coordinates_to_angles()
calib = {
    'pan_pixels': [],   # pixel-x values
    'pan_angles': [],   # corresponding pan servo angles
    'tilt_pixels': [],  # pixel-y values
    'tilt_angles': [],  # corresponding tilt servo angles
}


# === SERIAL ===
def try_open_serial(port, baud, retries=SERIAL_RETRIES, delay=SERIAL_RECONNECT_DELAY):
    global ser
    for i in range(retries):
        try:
            print(f"Serial try {i+1}/{retries}...")
            if ser and ser.is_open:
                ser.close()
            ser = serial.Serial(port, baud, timeout=1)
            time.sleep(0.2)
            if ser.is_open:
                print(f"Serial linked on {port}")
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                return ser
        except Exception as e:
            print(f"Serial fail: {e}")
            time.sleep(delay)
    print("No serial connection.")
    ser = None
    return None


def send_command_to_esp32(pan, tilt, force=False):
    global ser, last_sent_pan, last_sent_tilt
    if ser is None or not getattr(ser, 'is_open', False):
        try_open_serial(SERIAL_PORT, SERIAL_BAUD)
    if ser is None:
        return
    try:
        pan_i = int(pan)
        tilt_i = int(tilt)
        should_send = (
            force
            or last_sent_pan is None
            or last_sent_tilt is None
            or abs(pan_i - last_sent_pan) >= SEND_THRESHOLD
            or abs(tilt_i - last_sent_tilt) >= SEND_THRESHOLD
        )
        if should_send:
            cmd = f";vt {pan_i} {tilt_i}\r\n"
            ser.write(cmd.encode())
            last_sent_pan = pan_i
            last_sent_tilt = tilt_i
            time.sleep(0.01)
            try:
                if ser.in_waiting:
                    resp = ser.readline().decode(errors='ignore').strip()
                    if resp:
                        print(f"ESP32: {resp}")
            except Exception:
                pass
    except Exception as e:
        print(f"Serial error: {e}")
        try:
            ser.close()
        except Exception:
            pass
        ser = None


# === CAMERA ===
def open_capture(url):
    cap = cv2.VideoCapture(url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    return cap if cap.isOpened() else None


def try_open_with_retries(url, retries=3, delay=RECONNECT_DELAY):
    for _ in range(retries):
        cap = open_capture(url)
        if cap:
            print("Camera online.")
            return cap
        time.sleep(delay)
    return None


# === MATH ===
def predict_position(cur, prev, steps):
    if prev is None:
        return cur
    vx = cur[0] - prev[0]
    vy = cur[1] - prev[1]
    x = int(np.clip(cur[0] + vx * steps, 0, CAM_WIDTH))
    y = int(np.clip(cur[1] + vy * steps, 0, CAM_HEIGHT))
    return x, y


def map_coordinates_to_angles(x, y):
    """Map a pixel (x, y) to (pan, tilt) angles.

    Uses calibration data when available so that mechanical offsets and
    camera-FOV mismatches are corrected automatically.  Falls back to a
    simple linear mapping when calibration has not been run.
    """
    if len(calib['pan_pixels']) >= 2:
        pan = float(np.interp(x, calib['pan_pixels'], calib['pan_angles']))
    else:
        pan = float(np.interp(x, [0, CAM_WIDTH], [PAN_MIN_ANGLE, PAN_MAX_ANGLE]))

    if len(calib['tilt_pixels']) >= 2:
        tilt = float(np.interp(y, calib['tilt_pixels'], calib['tilt_angles']))
    else:
        # y=0 is top of frame; invert so looking up = higher tilt angle
        tilt = float(np.interp(y, [0, CAM_HEIGHT], [TILT_MAX_ANGLE, TILT_MIN_ANGLE]))

    return (
        int(np.clip(pan,  PAN_MIN_ANGLE,  PAN_MAX_ANGLE)),
        int(np.clip(tilt, TILT_MIN_ANGLE, TILT_MAX_ANGLE)),
    )


def contour_center(cnt):
    x, y, w, h = cv2.boundingRect(cnt)
    return (x + w // 2, y + h // 2)


# === CALIBRATION ===
_cal_click = None


def _on_cal_click(event, x, y, flags, param):
    global _cal_click
    if event == cv2.EVENT_LBUTTONDOWN:
        _cal_click = (x, y)


def run_calibration(cap):
    """Interactive startup calibration.

    The turret moves to four known angles one at a time.  For each position
    the user clicks where the aiming point actually lands in the camera feed,
    then confirms with SPACE.  This produces four (pixel → angle) samples
    that are stored in `calib` and used by map_coordinates_to_angles().

    Press ESC at any step to abort calibration and fall back to the default
    linear mapping.
    """
    global _cal_click, calib

    win = 'SkyGuard - Calibration'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, _on_cal_click)

    print()
    print("=" * 50)
    print("  CALIBRATION MODE")
    print("=" * 50)
    print("The turret will move to four positions.")
    print("For each one: click where the aiming point")
    print("lands in the camera view, then press SPACE.")
    print("Press ESC to skip and use default mapping.")
    print("=" * 50)
    print()

    for step_idx, (pan_angle, tilt_angle, axis, label) in enumerate(CAL_STEPS):
        # Move turret to the calibration position and wait for servo to settle
        send_command_to_esp32(pan_angle, tilt_angle, force=True)
        time.sleep(0.9)

        _cal_click = None
        confirmed = False
        aborted = False

        print(f"Step {step_idx + 1}/{len(CAL_STEPS)}: {label}  "
              f"(PAN={pan_angle}° TILT={tilt_angle}°)")
        print("  -> Click the aiming dot, then press SPACE to confirm.")

        while not confirmed:
            ret, frame = cap.read()
            if not ret:
                continue
            frame = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT))
            disp = frame.copy()

            cv2.putText(disp, f"CALIBRATION  Step {step_idx + 1}/{len(CAL_STEPS)}: {label}",
                        (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            cv2.putText(disp, f"PAN={pan_angle}  TILT={tilt_angle}",
                        (5, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            cv2.putText(disp, "Click aiming point then SPACE. ESC = skip calibration.",
                        (5, CAM_HEIGHT - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 180, 180), 1)

            if _cal_click:
                cv2.drawMarker(disp, _cal_click, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(disp, f"Marked {_cal_click} — SPACE to confirm",
                            (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)

            cv2.imshow(win, disp)
            key = cv2.waitKey(1) & 0xFF

            if key == 32 and _cal_click:   # SPACE — confirm
                confirmed = True
            elif key == 27:                # ESC — skip entire calibration
                aborted = True
                break

        if aborted:
            print("Calibration skipped — using default linear angle mapping.")
            cv2.destroyWindow(win)
            return

        px, py = _cal_click
        if axis == 'pan':
            calib['pan_pixels'].append(px)
            calib['pan_angles'].append(pan_angle)
            print(f"  Recorded pan:  pixel_x={px} -> {pan_angle} deg")
        else:
            calib['tilt_pixels'].append(py)
            calib['tilt_angles'].append(tilt_angle)
            print(f"  Recorded tilt: pixel_y={py} -> {tilt_angle} deg")

    cv2.destroyWindow(win)

    # Sort samples by pixel value so np.interp works correctly
    if len(calib['pan_pixels']) >= 2:
        pairs = sorted(zip(calib['pan_pixels'], calib['pan_angles']))
        calib['pan_pixels'], calib['pan_angles'] = map(list, zip(*pairs))
        print(f"Pan  calibration points: {list(zip(calib['pan_pixels'], calib['pan_angles']))}")

    if len(calib['tilt_pixels']) >= 2:
        pairs = sorted(zip(calib['tilt_pixels'], calib['tilt_angles']))
        calib['tilt_pixels'], calib['tilt_angles'] = map(list, zip(*pairs))
        print(f"Tilt calibration points: {list(zip(calib['tilt_pixels'], calib['tilt_angles']))}")

    print("Calibration complete!\n")


# === CLEANUP ===
def cleanup():
    global cap, ser
    if 'cap' in globals() and cap:
        cap.release()
    cv2.destroyAllWindows()
    if ser and getattr(ser, 'is_open', False):
        ser.close()
    print("Cleanup done. SkyGuard out.")


atexit.register(cleanup)

# === STARTUP ===
cap = try_open_with_retries(URL, retries=3)
if cap is None:
    print("Camera not found. Aborting.")
    sys.exit(1)

try_open_serial(SERIAL_PORT, SERIAL_BAUD)
run_calibration(cap)

back_sub = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=25, detectShadows=False)
print("SkyGuard active.  Keys: Q=quit  R=reset lock  L=lock on current target")

# === TRACKING STATE ===
prev_center = None
locked = False
locked_center = None
locked_bbox = None
lost_frames = 0
warmup_counter = 0

# === MAIN LOOP ===
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            cap.release()
            print("Reconnecting camera...")
            cap = try_open_with_retries(URL, retries=3)
            if cap is None:
                print("Camera lost. Exiting.")
                break
            prev_center = None
            warmup_counter = 0
            continue

        frame = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT))
        mask = back_sub.apply(frame)
        _, mask = cv2.threshold(mask, 244, 255, cv2.THRESH_BINARY)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=4)

        # Let the background model stabilise before tracking
        if warmup_counter < WARMUP_FRAMES:
            warmup_counter += 1
            cv2.putText(frame, f"Initialising... {warmup_counter}/{WARMUP_FRAMES}",
                        (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.imshow('SkyGuard - Live', frame)
            cv2.imshow('Mask', mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = sorted(
            [(cv2.contourArea(c), c) for c in contours if cv2.contourArea(c) > MIN_CONTOUR_AREA],
            key=lambda item: item[0],
            reverse=True,
        )

        if valid_contours:
            if locked and locked_center is not None:
                # Keep tracking the contour closest to where we last saw the target
                target_cnt = min(
                    valid_contours,
                    key=lambda item: (
                        (contour_center(item[1])[0] - locked_center[0]) ** 2 +
                        (contour_center(item[1])[1] - locked_center[1]) ** 2
                    ),
                )[1]
            else:
                # Auto-lock onto the largest moving object
                target_cnt = valid_contours[0][1]
                locked = True
                lost_frames = 0

            x, y, w, h = cv2.boundingRect(target_cnt)
            raw = (x + w // 2, y + h // 2)

            smooth = raw if prev_center is None else (
                int(prev_center[0] * (1 - SMOOTHING_ALPHA) + raw[0] * SMOOTHING_ALPHA),
                int(prev_center[1] * (1 - SMOOTHING_ALPHA) + raw[1] * SMOOTHING_ALPHA),
            )

            locked_center = smooth
            locked_bbox = (x, y, w, h)
            lost_frames = 0

            pred = predict_position(smooth, prev_center, PREDICTION_STEPS)
            pan, tilt = map_coordinates_to_angles(pred[0], pred[1])
            send_command_to_esp32(pan, tilt)

            print(f"Target {smooth} -> aim {pred} | PAN {pan}  TILT {tilt}")
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, smooth, 5, (0, 255, 0), -1)
            cv2.circle(frame, pred, 8, (0, 0, 255), 3)
            prev_center = smooth

        else:
            if locked and locked_center is not None:
                lost_frames += 1

                if lost_frames <= LOCK_HOLD_FRAMES:
                    # Target recently lost — keep predicting
                    pred = predict_position(locked_center, prev_center, PREDICTION_STEPS)
                    pan, tilt = map_coordinates_to_angles(pred[0], pred[1])
                    send_command_to_esp32(pan, tilt)
                    print(f"Predicting ({lost_frames}/{LOCK_HOLD_FRAMES}) -> {pan} / {tilt}")

                elif lost_frames <= LOCK_HOLD_FRAMES + HOLD_TIMEOUT_FRAMES:
                    # Hold last known position
                    pan, tilt = map_coordinates_to_angles(locked_center[0], locked_center[1])
                    hold_n = lost_frames - LOCK_HOLD_FRAMES
                    send_command_to_esp32(pan, tilt, force=False)
                    print(f"Holding ({hold_n}/{HOLD_TIMEOUT_FRAMES}) -> {pan} / {tilt}")

                else:
                    # Timed out — release lock and resume sweep
                    locked = False
                    locked_center = None
                    locked_bbox = None
                    lost_frames = 0
                    prev_center = None
                    print("Lock timed out — resuming sweep")

            else:
                # No target — sweep back and forth
                pan_sweep_angle += sweep_direction * SWEEP_INCREMENT
                if pan_sweep_angle >= SWEEP_MAX:
                    sweep_direction = -1
                    pan_sweep_angle = float(SWEEP_MAX)
                elif pan_sweep_angle <= SWEEP_MIN:
                    sweep_direction = 1
                    pan_sweep_angle = float(SWEEP_MIN)

                send_command_to_esp32(pan_sweep_angle, SWEEP_TILT, force=True)
                print(f"Sweeping: {pan_sweep_angle:.1f} deg (tilt {SWEEP_TILT})")
                prev_center = None

        cv2.imshow('SkyGuard - Live', frame)
        cv2.imshow('Mask', mask)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            locked = False
            locked_center = None
            locked_bbox = None
            lost_frames = 0
            prev_center = None   # reset so prediction doesn't jump
            print("Lock released")
        elif key == ord('l'):
            if prev_center is not None:
                locked = True
                locked_center = prev_center
                lost_frames = 0
                print("Manually locked")

except Exception as e:
    print(f"Loop crashed: {e}")
    traceback.print_exc()
finally:
    pass
