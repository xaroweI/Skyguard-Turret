import cv2
import numpy as np
import time
import sys
import serial
import atexit

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
LOCK_HOLD_FRAMES = 300


SERIAL_PORT = 'COM5'
SERIAL_BAUD = 115200
SERIAL_RETRIES = 3
SERIAL_RECONNECT_DELAY = 2.0

sweep_direction = -1
pan_sweep_angle = SWEEP_MAX

ser = None
last_sent_pan = None
last_sent_tilt = None

# === SERIAL STUFF ===
def try_open_serial(port, baud, retries=SERIAL_RETRIES, delay=SERIAL_RECONNECT_DELAY):
    global ser
    for i in range(retries):
        try:
            print(f"Serial try {i+1}/{retries}...")
            if ser and ser.is_open: ser.close()
            ser = serial.Serial(port, baud, timeout=1) 
            time.sleep(0.2)
            if ser.is_open:
                print(f"✅ Serial linked on {port}")
                ser.reset_input_buffer(); ser.reset_output_buffer()
                return ser
        except Exception as e:
            print(f"Serial fail: {e}")
            time.sleep(delay)
    print("💀 No serial vibes.")
    ser = None
    return None

def send_command_to_esp32(pan, tilt, force=False):
    global ser
    global last_sent_pan, last_sent_tilt
    if ser is None or not getattr(ser, 'is_open', False):
        try_open_serial(SERIAL_PORT, SERIAL_BAUD)
    if ser is None:
        print("⚠️ Serial ghosted. Skipping.")
        return
    try:
        pan_i = int(pan)
        tilt_i = int(tilt)

        send = False
        if last_sent_pan is None or last_sent_tilt is None:
            send = True
        elif abs(pan_i - last_sent_pan) >= SEND_THRESHOLD or abs(tilt_i - last_sent_tilt) >= SEND_THRESHOLD:
            send = True

        if send or force:
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
        else:
            return
    except Exception as e:
        print(f"Serial yeet: {e}")
        try: ser.close()
        except: pass
        ser = None

# === CAMERA & MATH ===
def open_capture(url):
    cap = cv2.VideoCapture(url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    return cap if cap.isOpened() else None

def try_open_with_retries(url, retries=3, delay=RECONNECT_DELAY):
    for _ in range(retries):
        cap = open_capture(url)
        if cap: 
            print("✅ Camera online.")
            return cap
        time.sleep(delay)
    return None

def predict_position(cur, prev, steps):
    if prev is None: return cur
    vx, vy = cur[0]-prev[0], cur[1]-prev[1]
    x = np.clip(cur[0]+vx*steps, 0, CAM_WIDTH)
    y = np.clip(cur[1]+vy*steps, 0, CAM_HEIGHT)
    return int(x), int(y)

def map_coordinates_to_angles(x, y):
    pan = np.interp(x, [0, CAM_WIDTH], [PAN_MIN_ANGLE, PAN_MAX_ANGLE])
    tilt = np.interp(y, [0, CAM_HEIGHT], [TILT_MAX_ANGLE, TILT_MIN_ANGLE])
    return int(np.clip(pan, PAN_MIN_ANGLE, PAN_MAX_ANGLE)), int(np.clip(tilt, TILT_MIN_ANGLE, TILT_MAX_ANGLE))

def cleanup():
    global cap, ser
    if 'cap' in globals() and cap: cap.release()
    cv2.destroyAllWindows()
    if ser and getattr(ser, 'is_open', False): ser.close()
    print("🫡 Cleanup done. Skyguard out.")

atexit.register(cleanup)

# === MAIN LOOP ===
cap = try_open_with_retries(URL, retries=3)
if cap is None:
    print("☠️ Camera MIA. Aborting.")
    sys.exit(1)

try_open_serial(SERIAL_PORT, SERIAL_BAUD)
back_sub = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=25, detectShadows=True)
print("🎯 Skyguard active.")

prev_center = None
locked = False
locked_center = None
locked_bbox = None
lost_frames = 0
MAX_LOST_FRAMES = LOCK_HOLD_FRAMES

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            cap.release()
            print("📸 Reconnecting cam...")
            cap = try_open_with_retries(URL, retries=3)
            if cap is None:
                print("💔 Cam dead. Bye.")
                break
            prev_center = None
            continue

        frame = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT))
        mask = back_sub.apply(frame)
        _, mask = cv2.threshold(mask, 244, 255, cv2.THRESH_BINARY)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=4)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > MIN_CONTOUR_AREA:
                valid_contours.append((area, cnt))

        valid_contours.sort(key=lambda x: x[0], reverse=True)

        if len(valid_contours) > 0:
            if locked and locked_center is not None:
                best = None
                best_dist = None
                for area, cnt in valid_contours:
                    x, y, w, h = cv2.boundingRect(cnt)
                    c = (x + w // 2, y + h // 2)
                    dist = (c[0]-locked_center[0])**2 + (c[1]-locked_center[1])**2
                    if best is None or dist < best_dist:
                        best, best_dist = (cnt, c), dist
                target_cnt, target_centroid = best
            else:
                target_cnt = valid_contours[0][1]
                x, y, w, h = cv2.boundingRect(target_cnt)
                target_centroid = (x + w // 2, y + h // 2)
                if not locked:
                    locked = True
                    locked_center = target_centroid
                    locked_bbox = (x, y, w, h)
                    lost_frames = 0

            x, y, w, h = cv2.boundingRect(target_cnt)
            raw = (x + w // 2, y + h // 2)
            if prev_center is None:
                smooth = raw
            else:
                smooth = (int(prev_center[0]*(1-SMOOTHING_ALPHA)+raw[0]*SMOOTHING_ALPHA),
                          int(prev_center[1]*(1-SMOOTHING_ALPHA)+raw[1]*SMOOTHING_ALPHA))

            locked_center = smooth
            locked_bbox = (x, y, w, h)
            lost_frames = 0

            pred = predict_position(smooth, prev_center, PREDICTION_STEPS)
            pan, tilt = map_coordinates_to_angles(pred[0], pred[1])
            send_command_to_esp32(pan, tilt)

            print(f"🎯 Locked Target {smooth} → AIM {pred} | PAN {pan}° / TILT {tilt}°")
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
            cv2.circle(frame, smooth, 5, (0,255,0), -1)
            cv2.circle(frame, pred, 8, (0,0,255), 3)
            prev_center = smooth

        else:
            if locked and locked_center is not None:
                if lost_frames < MAX_LOST_FRAMES:
                    lost_frames += 1
                    pred = predict_position(locked_center, prev_center, PREDICTION_STEPS)
                    pan, tilt = map_coordinates_to_angles(pred[0], pred[1])
                    send_command_to_esp32(pan, tilt)
                    print(f"🔒 Maintaining lock — no contour ({lost_frames}) — aiming {pan}°/{tilt}°")
                else:
                    pan, tilt = map_coordinates_to_angles(locked_center[0], locked_center[1])
                    send_command_to_esp32(pan, tilt, force=True) 
                    print(f"🔒 HOLDING last position (timeout) — aiming {pan}°/{tilt}°")
            
            else:
                pan_sweep_angle += sweep_direction * SWEEP_INCREMENT

                if pan_sweep_angle >= SWEEP_MAX:
                    sweep_direction = -1
                    pan_sweep_angle = SWEEP_MAX
                elif pan_sweep_angle <= SWEEP_MIN:
                    sweep_direction = 1
                    pan_sweep_angle = SWEEP_MIN

                send_command_to_esp32(pan_sweep_angle, SWEEP_TILT, force=True)
                print(f"🔍 Looking... SWEEP: {pan_sweep_angle:.1f}° (tilt {SWEEP_TILT}°)")
                prev_center = None

        cv2.imshow('Skyguard - Live', frame)
        cv2.imshow('Mask', mask)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            locked = False
            locked_center = None
            locked_bbox = None
            lost_frames = 0
            print('🔓 Lock released by user')
        elif key == ord('l'):
            if prev_center is not None:
                locked = True
                locked_center = prev_center
                print('🔒 Locked to current object')

except Exception as e:
    print(f"💣 Loop crashed: {e}")
finally:
    pass