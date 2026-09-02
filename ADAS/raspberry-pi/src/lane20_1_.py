import cv2
import numpy as np
from collections import deque
from smbus2 import SMBus, i2c_msg
import struct
import time
from picamera2 import Picamera2
from ultralytics import YOLO
from enum import Enum

# =====================================
# FINITE STATE MACHINE
# =====================================
class DrivingState(Enum):
    KEEP_LANE = 1
    CHANGE_LEFT = 2
    CHANGE_RIGHT = 3
    EMERGENCY_STOP = 4

# =====================================
# I2C CONFIG
# =====================================
I2C_BUS = 1
STM_ADDR = 0x12
IMU_ADDR = 0x6B          # ISM330DHCX I2C address (0x6B when SA0 is high)
bus = SMBus(I2C_BUS)

# =====================================
# IMU REGISTER MAP (ISM330DHCX)
# =====================================
WHO_AM_I_REG      = 0x0F
WHO_AM_I_EXPECTED = 0x6B   # ISM330DHCX WHO_AM_I value
CTRL1_XL_REG      = 0x10   # Accelerometer control
CTRL2_G_REG       = 0x11   # Gyroscope control
OUTX_L_G_REG      = 0x22   # Gyro output start  (6 bytes: Xl,Xh,Yl,Yh,Zl,Zh)
OUTX_L_A_REG      = 0x28   # Accel output start (6 bytes: Xl,Xh,Yl,Yh,Zl,Zh)

# Scale factors matching config below (±4g accel, ±250 dps gyro)
ACCEL_SCALE       = 4.0   / 32768.0   # g per LSB
GYRO_SCALE        = 250.0 / 32768.0   # dps per LSB

# =====================================
# COMPLEMENTARY FILTER PARAMETERS
# =====================================
CF_ALPHA          = 0.98   # Trust gyro this much (0=full accel, 1=full gyro)
# Smaller ALPHA → faster response to accel, more noise
# Larger  ALPHA → smoother angle, slower to correct gyro drift

# =====================================
# IMU CALIBRATION PARAMETERS
# =====================================
CALIB_SAMPLES     = 200    # Number of samples for bias estimation
CALIB_DELAY       = 0.01   # Seconds between calibration samples

# =====================================
# SPEED PARAMETERS
# =====================================
BASE_RPM = 30
MIN_RPM = 30
MAX_RPM = 30
LANE_CHANGE_RPM = 20

# =====================================
# CONTROLLER PARAMETERS (TUNED)
# =====================================
KP = 0.4
KD = 0.7

# IMU feed-forward gain: adds gyro yaw-rate correction to steering
KP_YAW = 0.08   # deg of extra steer per deg/s of yaw rate

MAX_STEER      = 45
MAX_STEER_STEP = 6

# =====================================
# LANE CHANGE PARAMETERS
# =====================================
LANE_CHANGE_STEP                  = 65
LANE_CHANGE_COOLDOWN              = 30
LANE_CHANGE_COMPLETION_THRESHOLD  = 25
LANE_CHANGE_STABILIZATION_FRAMES  = 3

# =====================================
# OBSTACLE DETECTION PARAMETERS
# =====================================
DANGER_ZONE_WIDTH_RATIO        = 0.3
DANGER_ZONE_HEIGHT_START       = 0.3
DANGER_ZONE_HEIGHT_END         = 0.7
OBSTACLE_CONFIDENCE_THRESHOLD  = 0.5
CONSECUTIVE_DETECTION_FRAMES   = 1

OBSTACLE_CLASSES = [0, 1, 2, 3, 5, 7]  # person, bicycle, car, motorcycle, bus, truck

# =====================================
# TRACKING & DISTANCE PARAMETERS
# =====================================
# Real-world object heights in metres (approximate) used for distance estimation
# Formula: distance = (real_height_m * FOCAL_LENGTH_PX) / bbox_height_px
REAL_HEIGHTS_M = {
    0: 1.7,   # person
    1: 1.1,   # bicycle
    2: 1.5,   # car
    3: 1.1,   # motorcycle
    5: 3.2,   # bus
    7: 2.8,   # truck
}
# Focal length in pixels – calibrate for your camera.
# Approximation: focal_px ≈ (image_width / 2) / tan(half_hfov)
# For Pi Camera v2 (62° HFOV) at 640 px wide: ~554 px
FOCAL_LENGTH_PX = 554.0

# Tracker parameters
TRACKER_MAX_DIST   = 80    # max centroid distance (px) to match same object
TRACKER_MAX_LOST   = 10   # frames before a track is dropped

# Speed estimation
SPEED_HISTORY      = 5    # frames to average speed over

# Forward Collision Warning
FCW_AREA_GROWTH_THRESHOLD = 0.15  # bbox area must grow >15%/frame to trigger
FCW_MIN_DISTANCE_M        = 3.0   # only warn if object closer than this (m)
FCW_MIN_FRAMES            = 3     # consecutive growing frames before warning

# =====================================
# CAMERA SETTINGS
# =====================================
WIDTH  = 640
HEIGHT = 360

LANE_WIDTH = 280
HALF_LANE  = LANE_WIDTH // 2

DEBUG_WINDOWS = True

# =====================================
# LANE DETECTION CONSTANTS  (all tunable here)
# =====================================
# Canny edge thresholds
CANNY_LOW           = 50    # raise if too much noise; lower if missing edges
CANNY_HIGH          = 150

# HoughLinesP
HOUGH_THRESHOLD     = 30    # minimum votes to accept a line
HOUGH_MIN_LEN       = 40    # minimum segment length (px)
HOUGH_MAX_GAP       = 100   # max gap to bridge between segments (px)

# Slope band — lines outside these ranges are discarded
SLOPE_LEFT_MIN      = -2.5  # left lane  (negative slope in image coords)
SLOPE_LEFT_MAX      = -0.4
SLOPE_RIGHT_MIN     =  0.4  # right lane (positive slope)
SLOPE_RIGHT_MAX     =  2.5

# Temporal smoothing (exponential moving average) for lane center
# 0 = no smoothing, 1 = never updates.  0.5–0.7 works well on Pi
EMA_ALPHA           = 0.6

# Steering deadband: errors smaller than this (px) are treated as zero
# Reduces jitter when car is roughly centred
STEER_DEADBAND_PX   = 12

# Steering bias offset: positive shifts target right, negative shifts left
# Use to correct persistent drift without changing PD gains
STEER_BIAS_PX       = 0     # start at 0, tune if car still drifts

# Steering clamp: maximum pixel error fed to PD controller
# Prevents extreme oscillation on large jumps
STEER_MAX_ERROR_PX  = 160

# Opposite-lane spike rejection
# If error changes by more than this in one frame → treat as spike
SPIKE_THRESHOLD_PX  = 40   # lower = more sensitive


# =====================================
# GLOBAL STATE VARIABLES
# =====================================
current_state             = DrivingState.KEEP_LANE
target_lane_center        = WIDTH // 2
lane_change_cooldown_counter = 0
last_error                = 0
last_steer                = 0

error_buffer      = deque(maxlen=5)
lane_center_buffer = deque(maxlen=6)

# Spike rejection state
prev_raw_error = 0
spike_active   = False


# =====================================
# IMU DRIVER
# =====================================
class IMU:
    """
    Driver for ISM330DHCX at I2C address 0x6B (pure smbus2).
    Provides calibrated gyro + accel readings and runs a
    complementary filter to track roll, pitch, and yaw.

    Config: Accel ±4g @ 104 Hz  |  Gyro ±250 dps @ 104 Hz
    """

    def __init__(self, bus: SMBus, addr: int = IMU_ADDR):
        self.bus  = bus
        self.addr = addr

        # Calibration offsets (filled by calibrate())
        self.gyro_bias  = np.zeros(3)   # [gx, gy, gz] dps
        self.accel_bias = np.zeros(3)   # [ax, ay, az] g  (z bias should stay ~1 g)

        # Complementary filter angles [roll, pitch, yaw] in degrees
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0   # NOTE: yaw from gyro only (no magnetometer), drifts over time

        # Yaw rate (deg/s) – useful for feed-forward steering
        self.yaw_rate = 0.0

        self._last_time = time.time()
        self._initialized = False

    # ------------------------------------------------------------------
    def begin(self):
        """Verify chip identity, configure ODR and full-scale ranges."""
        # Verify chip identity
        who = bus.read_byte_data(self.addr, WHO_AM_I_REG)
        print(f"[IMU] WHO_AM_I: 0x{who:02X} (expected 0x{WHO_AM_I_EXPECTED:02X})")
        if who != WHO_AM_I_EXPECTED:
            raise RuntimeError(
                f"ISM330DHCX not found at 0x{self.addr:02X}. "
                f"Got WHO_AM_I=0x{who:02X}"
            )

        # CTRL1_XL: ODR=104 Hz (0x4_), ±4g full-scale (0x_8 → FS=10)
        # Bits [7:4]=0100 → 104 Hz | Bits [3:2]=10 → ±4g
        bus.write_byte_data(self.addr, CTRL1_XL_REG, 0x48)

        # CTRL2_G: ODR=104 Hz (0x4_), ±250 dps (0x_0 → FS=000)
        # Bits [7:4]=0100 → 104 Hz | Bits [3:1]=000 → ±250 dps
        bus.write_byte_data(self.addr, CTRL2_G_REG, 0x40)

        time.sleep(0.1)   # allow sensor to settle
        self._initialized = True
        print("[IMU] ISM330DHCX configured: Accel ±4g @ 104 Hz, Gyro ±250 dps @ 104 Hz")

    # ------------------------------------------------------------------
    def _read_raw(self):
        """Read 6 gyro + 6 accel bytes via smbus2. Returns signed int16 tuples."""
        # Read 6 bytes starting at gyro output register (auto-increment)
        g_raw = bus.read_i2c_block_data(self.addr, OUTX_L_G_REG, 6)
        a_raw = bus.read_i2c_block_data(self.addr, OUTX_L_A_REG, 6)

        # Combine low/high bytes and sign-extend to int16
        def to_int16(lo, hi):
            val = (hi << 8) | lo
            return val - 65536 if val >= 32768 else val

        gx = to_int16(g_raw[0], g_raw[1])
        gy = to_int16(g_raw[2], g_raw[3])
        gz = to_int16(g_raw[4], g_raw[5])

        ax = to_int16(a_raw[0], a_raw[1])
        ay = to_int16(a_raw[2], a_raw[3])
        az = to_int16(a_raw[4], a_raw[5])

        # IMU is mounted upside-down → invert X axis for both gyro and accel
        gx = -gx
        ax = -ax

        return (gx, gy, gz), (ax, ay, az)

    # ------------------------------------------------------------------
    def calibrate(self, samples: int = CALIB_SAMPLES, delay: float = CALIB_DELAY):
        """
        Collect 'samples' readings with the robot stationary to estimate
        gyro and accel biases. Keep the chassis flat and still!
        """
        if not self._initialized:
            self.begin()

        print(f"[IMU] Calibrating – keep vehicle STILL for {samples * delay:.1f} s …")

        gyro_acc  = np.zeros(3)
        accel_acc = np.zeros(3)

        for i in range(samples):
            (gx, gy, gz), (ax, ay, az) = self._read_raw()
            gyro_acc  += [gx, gy, gz]
            accel_acc += [ax, ay, az]
            time.sleep(delay)
            if (i + 1) % 50 == 0:
                print(f"  … {i + 1}/{samples}")

        self.gyro_bias  = gyro_acc  / samples * GYRO_SCALE
        self.accel_bias = accel_acc / samples * ACCEL_SCALE
        # For a flat sensor the Z accel should read +1 g – keep that offset
        # so we subtract only XY accel bias; Z stays referenced to gravity.
        self.accel_bias[2] -= 1.0   # remove gravity contribution from Z bias

        print(f"[IMU] Gyro  bias (dps): gx={self.gyro_bias[0]:+.4f}  "
              f"gy={self.gyro_bias[1]:+.4f}  gz={self.gyro_bias[2]:+.4f}")
        print(f"[IMU] Accel bias  (g) : ax={self.accel_bias[0]:+.4f}  "
              f"ay={self.accel_bias[1]:+.4f}  az={self.accel_bias[2]:+.4f}")
        print("[IMU] Calibration complete.")

        # Reset filter angles
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        self._last_time = time.time()

    # ------------------------------------------------------------------
    def update(self):
        """
        Read sensor, apply calibration, run complementary filter.
        Call this once per main-loop iteration.

        Complementary filter equations
        ────────────────────────────────
          angle_accel_roll  = atan2(ay,  az)   [roll from accel]
          angle_accel_pitch = atan2(-ax, az)   [pitch from accel]

          angle_gyro  += gyro_rate * dt         [integrate gyro]

          angle = α * angle_gyro + (1-α) * angle_accel

        Yaw is gyro-only (no accel reference for horizontal rotation):
          yaw += gz_calibrated * dt
        """
        if not self._initialized:
            return

        now = time.time()
        dt  = now - self._last_time
        self._last_time = now

        # Guard against zero / huge dt
        if dt <= 0 or dt > 0.5:
            return

        # ── Raw → physical units ──────────────────────────────────────
        (gx_r, gy_r, gz_r), (ax_r, ay_r, az_r) = self._read_raw()

        gx = gx_r * GYRO_SCALE  - self.gyro_bias[0]   # dps
        gy = gy_r * GYRO_SCALE  - self.gyro_bias[1]
        gz = gz_r * GYRO_SCALE  - self.gyro_bias[2]

        ax = ax_r * ACCEL_SCALE - self.accel_bias[0]   # g
        ay = ay_r * ACCEL_SCALE - self.accel_bias[1]
        az = az_r * ACCEL_SCALE - self.accel_bias[2]

        # ── Accel angles (degrees) ────────────────────────────────────
        roll_accel  = np.degrees(np.arctan2(ay, az))
        pitch_accel = np.degrees(np.arctan2(-ax, np.sqrt(ay**2 + az**2)))

        # ── Complementary filter ──────────────────────────────────────
        # Gyro integration
        roll_gyro  = self.roll  + gx * dt
        pitch_gyro = self.pitch + gy * dt

        # Fuse
        self.roll  = CF_ALPHA * roll_gyro  + (1.0 - CF_ALPHA) * roll_accel
        self.pitch = CF_ALPHA * pitch_gyro + (1.0 - CF_ALPHA) * pitch_accel

        # Yaw: integrate Z-gyro only
        self.yaw      += gz * dt
        self.yaw_rate  = gz          # instantaneous yaw rate (deg/s)

    # ------------------------------------------------------------------
    @property
    def angles(self):
        """Return (roll, pitch, yaw) in degrees."""
        return self.roll, self.pitch, self.yaw


# =====================================
# YOLO MODEL
# =====================================
# PyTorch 2.6 changed torch.load default to weights_only=True which breaks
# YOLO checkpoint loading. We patch it back to False before initializing.
import torch
import functools

_original_torch_load = torch.load

@functools.wraps(_original_torch_load)
def _patched_torch_load(f, *args, **kwargs):
    kwargs.setdefault('weights_only', False)
    return _original_torch_load(f, *args, **kwargs)

torch.load = _patched_torch_load

print("Loading YOLO model…")
yolo_model = YOLO('yolov8n.pt')
print("YOLO model loaded.")

# Restore original torch.load after model is loaded
torch.load = _original_torch_load


# =====================================
# SEND DATA TO STM32
# =====================================
def send_data(rpm, desired, actual):
    rpm     = int(max(0, min(rpm, 3000)))
    desired = int(max(0, min(desired, 180)))
    actual  = int(max(0, min(actual, 180)))
    payload = struct.pack('<HHH', rpm, desired, actual)
    msg = i2c_msg.write(STM_ADDR, payload)
    bus.i2c_rdwr(msg)


# =====================================
# ROI  — strictly symmetric around frame centre
# =====================================
def region_of_interest(img):
    """
    Symmetric trapezoid ROI derived purely from frame dimensions.
    Top edge is centred on w/2 and spans 12% of width each side.
    Bottom edge spans nearly full width.
    This prevents one side being wider than the other (a common
    cause of right-lane over-detection).
    """
    h, w = img.shape[:2]
    cx   = w // 2
    mask = np.zeros_like(img)
    poly = np.array([[
        (int(cx - w * 0.48), h),               # bottom-left
        (int(cx + w * 0.48), h),               # bottom-right
        (int(cx + w * 0.12), int(h * 0.42)),   # top-right
        (int(cx - w * 0.12), int(h * 0.42)),   # top-left
    ]], np.int32)
    cv2.fillPoly(mask, poly, 255)
    return cv2.bitwise_and(img, mask)


# =====================================
# PREPROCESS
# =====================================
def preprocess(frame):
    """Gray → Gaussian blur → Canny → symmetric ROI crop."""
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur  = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)
    roi   = region_of_interest(edges)
    return roi


# =====================================
# HOUGH LANE DETECTION
# =====================================

# Previous-frame lane memory (used when one side is missing)
_prev_left_lane  = None
_prev_right_lane = None


def _fit_lane(segs, h):
    """
    Length-weighted polyfit of accepted segments → one (x1,y1,x2,y2) line
    that runs from the bottom of the ROI to 60% up the frame.
    Weighting by segment length gives longer (more reliable) segments
    more influence than short noise segments.
    """
    xs, ys, ws = [], [], []
    for x1, y1, x2, y2 in segs:
        w = np.hypot(x2 - x1, y2 - y1)   # segment length as weight
        xs.extend([x1, x2])
        ys.extend([y1, y2])
        ws.extend([w,  w])
    poly = np.polyfit(ys, xs, 1, w=ws)   # x = a*y + b
    yb = h
    yt = int(h * 0.6)
    return (int(poly[0]*yb + poly[1]), yb,
            int(poly[0]*yt + poly[1]), yt)


def detect_lane_lines(edges, frame_shape):
    """
    Improvements applied
    ────────────────────
    1. Symmetric ROI (in region_of_interest above).
    2. Strict slope-band filtering per side — no shared threshold.
    3. Near-horizontal lines rejected (|slope| < SLOPE_RIGHT_MIN).
    4. Left and right averaged independently via _fit_lane().
    5. Missing lane estimated from previous frame memory.
    """
    global _prev_left_lane, _prev_right_lane

    h, w = frame_shape[:2]

    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=HOUGH_MIN_LEN,
        maxLineGap=HOUGH_MAX_GAP,
    )

    left_segs, right_segs = [], []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            if abs(dx) < 1:
                continue
            slope = (y2 - y1) / dx

            # Strictly separate left / right by slope band
            if SLOPE_LEFT_MIN <= slope <= SLOPE_LEFT_MAX:
                left_segs.append((x1, y1, x2, y2))
            elif SLOPE_RIGHT_MIN <= slope <= SLOPE_RIGHT_MAX:
                right_segs.append((x1, y1, x2, y2))
            # anything outside both bands (too flat or too vertical) → ignored

    # Fit each side independently
    left_lane  = _fit_lane(left_segs,  h) if left_segs  else None
    right_lane = _fit_lane(right_segs, h) if right_segs else None

    # ── Previous-frame fallback for missing lane ──────────────────────
    if left_lane is None and _prev_left_lane is not None:
        left_lane = _prev_left_lane      # hold last known position
    if right_lane is None and _prev_right_lane is not None:
        right_lane = _prev_right_lane

    # Update memory only when we have a fresh detection
    if left_segs:
        _prev_left_lane  = left_lane
    if right_segs:
        _prev_right_lane = right_lane

    return left_lane, right_lane


def get_lane_position(lane, y):
    if lane is None:
        return None
    x1, y1, x2, y2 = lane
    if y2 == y1:
        return x1
    return int(x1 + (x2 - x1) / (y2 - y1) * (y - y1))

# =====================================
# LANE INFO
# =====================================
class LaneInfo:
    def __init__(self):
        self.left_line           = None
        self.right_line          = None
        self.current_lane_center = None
        self.lane_width          = LANE_WIDTH
        self.left_lane_center    = None
        self.right_lane_center   = None

    def update(self, left_lane, right_lane, frame_width):
        # Store detected lines
        self.left_line  = left_lane
        self.right_line = right_lane

        h  = HEIGHT - 1
        lx = get_lane_position(left_lane,  h)
        rx = get_lane_position(right_lane, h)

        margin = frame_width // 8

        if lx is not None and rx is not None:
            # Compute real lane center from boundaries
            self.current_lane_center = (lx + rx) // 2

            # Update lane width safely
            w = rx - lx
            if 150 <= w <= 450:
                self.lane_width = w

            # Compute absolute neighboring lane centers
            self.left_lane_center  = lx + self.lane_width // 2
            self.right_lane_center = rx - self.lane_width // 2

        elif lx is not None:
            # Only left boundary detected
            self.current_lane_center = lx + self.lane_width // 2
            self.left_lane_center    = self.current_lane_center
            self.right_lane_center   = self.current_lane_center + self.lane_width

        elif rx is not None:
            # Only right boundary detected
            self.current_lane_center = rx - self.lane_width // 2
            self.right_lane_center   = self.current_lane_center
            self.left_lane_center    = self.current_lane_center - self.lane_width

        else:
            # No detection fallback
            if self.current_lane_center is None:
                self.current_lane_center = frame_width // 2

        # Clamp all centers safely within frame
        self.current_lane_center = max(
            margin,
            min(frame_width - margin, self.current_lane_center)
        )

        if self.left_lane_center is not None:
            self.left_lane_center = max(
                margin,
                min(frame_width - margin, self.left_lane_center)
            )

        if self.right_lane_center is not None:
            self.right_lane_center = max(
                margin,
                min(frame_width - margin, self.right_lane_center)
            )
# =====================================
# CENTROID OBJECT TRACKER
# =====================================
class Track:
    """Single tracked object."""
    _next_id = 0

    def __init__(self, centroid, bbox, cls):
        self.id            = Track._next_id
        Track._next_id    += 1
        self.centroid      = centroid        # (cx, cy)
        self.bbox          = bbox
        self.cls           = cls
        self.lost          = 0              # frames since last matched detection
        self.age           = 1             # total frames alive

        # For speed estimation: history of (timestamp, distance_m)
        self.dist_history  = deque(maxlen=SPEED_HISTORY)
        # For FCW: history of bbox areas
        self.area_history  = deque(maxlen=FCW_MIN_FRAMES + 1)
        # FCW consecutive growing frame counter
        self.fcw_counter   = 0
        self.fcw_active    = False

    def update(self, centroid, bbox):
        self.centroid = centroid
        self.bbox     = bbox
        self.lost     = 0
        self.age     += 1

# =====================================
# DISTANCE ESTIMATOR
# =====================================
class DistanceEstimator:
    """
    Pinhole camera model:
        distance = (real_height_m * focal_length_px) / bbox_height_px
    """
    def estimate(self, bbox, cls):
        x1, y1, x2, y2 = bbox
        bbox_h = max(5, y2 - y1)  # avoid division spikes
        real_h = REAL_HEIGHTS_M.get(cls, 1.5)
        distance = (real_h * FOCAL_LENGTH_PX) / bbox_h
        return max(0.2, min(50.0, distance))
class CentroidTracker:
    """
    Greedy nearest-centroid matching between frames.
    Returns a list of active Track objects each frame.
    """
    def __init__(self):
        self.tracks = []   # list of Track

    def update(self, detections):
        """
        detections: list of (bbox, cls, conf)
        Returns updated list of active Track objects.
        """
        # Build detection centroids
        det_centroids = []
        for (x1, y1, x2, y2), cls, conf in detections:
            det_centroids.append(((x1 + x2) // 2, (y1 + y2) // 2))

        matched_det  = set()
        matched_trk  = set()

        # Greedy matching: for each track find closest unmatched detection
        if self.tracks and det_centroids:
            for ti, track in enumerate(self.tracks):
                best_dist = TRACKER_MAX_DIST
                best_di   = -1
                for di, dc in enumerate(det_centroids):
                    if di in matched_det:
                        continue
                    d = np.hypot(track.centroid[0] - dc[0],
                                 track.centroid[1] - dc[1])
                    if d < best_dist:
                        best_dist = d
                        best_di   = di
                if best_di >= 0:
                    matched_det.add(best_di)
                    matched_trk.add(ti)
                    bbox = detections[best_di][0]
                    self.tracks[ti].update(det_centroids[best_di], bbox)

        # Increment lost counter for unmatched tracks
        for ti, track in enumerate(self.tracks):
            if ti not in matched_trk:
                track.lost += 1

        # Create new tracks for unmatched detections
        for di, (bbox, cls, conf) in enumerate(detections):
            if di not in matched_det:
                self.tracks.append(Track(det_centroids[di], bbox, cls))

        # Remove stale tracks
        self.tracks = [t for t in self.tracks if t.lost <= TRACKER_MAX_LOST]

        return self.tracks


# =====================================
# SPEED ESTIMATOR
# =====================================
class SpeedEstimator:
    """
    Estimates approach speed (m/s) of a tracked object using
    the change in estimated distance over time.
    Positive speed = approaching, negative = receding.
    """
    def update(self, track, distance_m, timestamp):
        track.dist_history.append((timestamp, distance_m))
        if len(track.dist_history) < 2:
            return 0.0
        t0, d0 = track.dist_history[0]
        t1, d1 = track.dist_history[-1]
        dt = t1 - t0
        if dt < 1e-4:
            return 0.0
        # Positive = object getting closer (distance decreasing)
        return (d0 - d1) / dt


# =====================================
# FORWARD COLLISION WARNING
# =====================================
class FCWSystem:
    """
    Monitors bbox area growth rate per track.
    Triggers warning when an object in the danger zone is:
      - closer than FCW_MIN_DISTANCE_M
      - AND its bounding box area grows > FCW_AREA_GROWTH_THRESHOLD for
        FCW_MIN_FRAMES consecutive frames
    """
    def update(self, track, distance_m, in_danger_zone):
        x1, y1, x2, y2 = track.bbox
        area = (x2 - x1) * (y2 - y1)
        track.area_history.append(area)

        if not in_danger_zone or distance_m > FCW_MIN_DISTANCE_M:
            track.fcw_counter = 0
            track.fcw_active  = False
            return False

        if len(track.area_history) >= 2:
            prev = track.area_history[-2]
            curr = track.area_history[-1]
            growth = (curr - prev) / max(1, prev)
            if growth > FCW_AREA_GROWTH_THRESHOLD:
                track.fcw_counter += 1
            else:
                track.fcw_counter = 0
        
        track.fcw_active = track.fcw_counter >= FCW_MIN_FRAMES
        return track.fcw_active


# =====================================
# OBSTACLE DETECTION
# =====================================
class ObstacleDetector:
    def __init__(self, frame_width, frame_height):
        self.frame_width  = frame_width
        self.frame_height = frame_height
        self.danger_x_min = int(frame_width  * (0.5 - DANGER_ZONE_WIDTH_RATIO / 2))
        self.danger_x_max = int(frame_width  * (0.5 + DANGER_ZONE_WIDTH_RATIO / 2))
        self.danger_y_min = int(frame_height * DANGER_ZONE_HEIGHT_START)
        self.danger_y_max = int(frame_height * DANGER_ZONE_HEIGHT_END)

    def detect_obstacles(self, frame):
        results = yolo_model(frame, verbose=False, conf=OBSTACLE_CONFIDENCE_THRESHOLD)
        obstacle_in_danger_zone = False
        all_detections = []
        for result in results:
            for box in result.boxes:
                cls  = int(box.cls[0])
                conf = float(box.conf[0])
                if cls not in OBSTACLE_CLASSES:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                bbox = (int(x1), int(y1), int(x2), int(y2))
                all_detections.append((bbox, cls, conf))
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                if (self.danger_x_min <= cx <= self.danger_x_max and
                        self.danger_y_min <= cy <= self.danger_y_max):
                    obstacle_in_danger_zone = True
        return obstacle_in_danger_zone, all_detections

    def is_in_danger_zone(self, bbox):
        x1, y1, x2, y2 = bbox
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        return (self.danger_x_min <= cx <= self.danger_x_max and
                self.danger_y_min <= cy <= self.danger_y_max)

    def draw_danger_zone(self, frame):
        cv2.rectangle(frame,
                      (self.danger_x_min, self.danger_y_min),
                      (self.danger_x_max, self.danger_y_max),
                      (0, 0, 255), 2)
        cv2.putText(frame, "DANGER ZONE",
                    (self.danger_x_min + 5, self.danger_y_min + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


# =====================================
# LANE OCCUPANCY CHECKER
# =====================================
class LaneOccupancyChecker:
    def __init__(self, frame_width):
        self.frame_width = frame_width

    def check_lane_occupancy(self, detections, lane_info):
        left_lane_free  = True
        right_lane_free = True
        if lane_info.current_lane_center is None:
            return False, False
        c = lane_info.current_lane_center
        w = lane_info.lane_width
        left_x_min  = max(0, c - 2 * w)
        left_x_max  = c - w // 2
        right_x_min = c + w // 2
        right_x_max = min(self.frame_width, c + 2 * w)
        for (x1, y1, x2, y2), cls, conf in detections:
            cx = (x1 + x2) / 2
            if left_x_min  <= cx <= left_x_max:
                left_lane_free  = False
            if right_x_min <= cx <= right_x_max:
                right_lane_free = False
        return left_lane_free, right_lane_free


# =====================================
# DECISION LOGIC (FSM)
# =====================================
class LaneChangeDecisionMaker:
    def __init__(self):
        self.consecutive_obstacle_count = 0

    def make_decision(self, current_state, obstacle_detected,
                      left_lane_free, right_lane_free, cooldown_counter):
        if cooldown_counter > 0:
            if current_state in (DrivingState.CHANGE_LEFT, DrivingState.CHANGE_RIGHT):
                return current_state, True
            return current_state, False

        if obstacle_detected:
            self.consecutive_obstacle_count += 1
        else:
            self.consecutive_obstacle_count = 0

        if self.consecutive_obstacle_count >= CONSECUTIVE_DETECTION_FRAMES:
            if left_lane_free and not right_lane_free:
                return DrivingState.CHANGE_LEFT, True
            elif right_lane_free and not left_lane_free:
                return DrivingState.CHANGE_RIGHT, True
            elif left_lane_free and right_lane_free:
                return DrivingState.CHANGE_LEFT, True
            else:
                return DrivingState.EMERGENCY_STOP, False

        if current_state in (DrivingState.CHANGE_LEFT, DrivingState.CHANGE_RIGHT):
            return current_state, True
        return DrivingState.KEEP_LANE, False


# =====================================
# SMOOTH LANE CHANGE CONTROLLER
# =====================================
class SmoothLaneChanger:
    def __init__(self, frame_width):
        self.frame_width          = frame_width
        self.target_center        = frame_width // 2
        self.lane_change_active   = False
        self.stabilization_counter = 0
        self.target_reached       = False

    def update_target(self, current_state, lane_info, current_lane_center):
        lane_change_complete = False

        if current_state == DrivingState.KEEP_LANE:
            desired_target = current_lane_center
            self.lane_change_active    = False
            self.stabilization_counter = 0
            self.target_reached        = False
        elif current_state == DrivingState.CHANGE_LEFT:
            desired_target = lane_info.left_lane_center
            self.lane_change_active = True
        elif current_state == DrivingState.CHANGE_RIGHT:
            desired_target = lane_info.right_lane_center
            self.lane_change_active = True
        else:
            desired_target = self.target_center
            self.lane_change_active    = False
            self.stabilization_counter = 0
            self.target_reached        = False

        diff = desired_target - self.target_center
        if abs(diff) > LANE_CHANGE_STEP:
            self.target_center += int(np.sign(diff)) * LANE_CHANGE_STEP
            self.target_reached        = False
            self.stabilization_counter = 0
        else:
            self.target_center = desired_target
            if self.lane_change_active and abs(diff) < LANE_CHANGE_COMPLETION_THRESHOLD:
                if not self.target_reached:
                    self.target_reached        = True
                    self.stabilization_counter = 0
                self.stabilization_counter += 1
                if self.stabilization_counter >= LANE_CHANGE_STABILIZATION_FRAMES:
                    lane_change_complete       = True
                    self.stabilization_counter = 0
                    self.target_reached        = False

        self.target_center = max(self.frame_width // 4,
                                 min(3 * self.frame_width // 4, self.target_center))
        return self.target_center, lane_change_complete


# =====================================
# RPM CONTROL
# =====================================
def compute_dynamic_rpm(abs_steer, current_state):
    if current_state == DrivingState.EMERGENCY_STOP:
        return 0
    if current_state in (DrivingState.CHANGE_LEFT, DrivingState.CHANGE_RIGHT):
        return LANE_CHANGE_RPM
    steer_factor = abs_steer / MAX_STEER
    rpm = BASE_RPM - steer_factor * (BASE_RPM - MIN_RPM)
    return int(max(MIN_RPM, min(MAX_RPM, rpm)))


# =====================================
# PD STEERING + IMU FEED-FORWARD + RATE LIMIT
# =====================================
def compute_steer(error, yaw_rate: float = 0.0):
    """
    PD controller with:
      6. EMA smoothing applied upstream (in main loop) before this call.
      7. Steering deadband  — small errors ignored to kill jitter.
      8. Bias offset        — correct persistent drift without retuning KP.
      9. Output clamp       — prevent extreme oscillation.
         Rate limiter       — cap per-frame angle change.
      IMU yaw-rate feed-forward — damp rotational disturbances.

    yaw_rate: calibrated gz in deg/s (positive = turning right)
    """
    global last_error, last_steer

    # 8. Apply bias offset before anything else
    error = error + STEER_BIAS_PX

    # 7. Deadband: treat tiny errors as zero to prevent jitter
    if abs(error) < STEER_DEADBAND_PX:
        error = 0

    # 9. Clamp large errors to prevent wild overcorrection
    error = max(-STEER_MAX_ERROR_PX, min(STEER_MAX_ERROR_PX, error))

    derivative  = error - last_error
    last_error  = error

    # PD term
    steer = KP * error + KD * derivative

    # IMU feed-forward
    steer -= KP_YAW * yaw_rate

    # Output clamp
    steer = max(-MAX_STEER, min(MAX_STEER, steer))

    # Rate limiter: cap per-frame change
    delta      = steer - last_steer
    delta      = max(-MAX_STEER_STEP, min(MAX_STEER_STEP, delta))
    steer      = last_steer + delta
    last_steer = steer
    return steer


# =====================================
# VISUALIZATION
# =====================================
def draw_comprehensive_info(frame, lane_info, target_center, error, angle, rpm,
                            fps, current_state, obstacle_detected,
                            left_free, right_free, stabilization_counter=0,
                            imu_roll=0.0, imu_pitch=0.0, imu_yaw=0.0, imu_yaw_rate=0.0):
    h, w = frame.shape[:2]
    car_center = w // 2

    if lane_info.current_lane_center:
        cv2.line(frame, (lane_info.current_lane_center, 0),
                 (lane_info.current_lane_center, h), (255, 0, 0), 2)
    cv2.line(frame, (target_center, 0), (target_center, h), (0, 255, 0), 2)
    cv2.line(frame, (car_center, 0),    (car_center, h),    (0, 0, 255), 2)
    cv2.arrowedLine(frame, (car_center, h // 2),
                    (target_center, h // 2), (0, 255, 255), 3)

    if lane_info.left_line:
        x1, y1, x2, y2 = lane_info.left_line
        cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 0), 3)
    if lane_info.right_line:
        x1, y1, x2, y2 = lane_info.right_line
        cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 0), 3)

    for lc in (lane_info.left_lane_center, lane_info.right_lane_center):
        if lc:
            for y in range(0, h, 20):
                cv2.line(frame, (lc, y), (lc, y + 10), (255, 200, 100), 1)

    font = cv2.FONT_HERSHEY_SIMPLEX
    yo   = 30
    lh   = 25

    state_colors = {
        DrivingState.KEEP_LANE:     (0, 255, 0),
        DrivingState.CHANGE_LEFT:   (255, 255, 0),
        DrivingState.CHANGE_RIGHT:  (255, 255, 0),
        DrivingState.EMERGENCY_STOP:(0, 0, 255),
    }
    state_text = current_state.name
    if stabilization_counter > 0:
        state_text += f" [STAB {stabilization_counter}/10]"

    cv2.putText(frame, f"State: {state_text}", (20, yo),
                font, 0.6, state_colors.get(current_state, (255, 255, 255)), 2); yo += lh
    cv2.putText(frame, f"Err:{error:+4d}", (20, yo), font, 0.6, (255, 255, 255), 2); yo += lh
    cv2.putText(frame, f"Angle:{angle}",   (20, yo), font, 0.6, (255, 255, 255), 2); yo += lh
    cv2.putText(frame, f"RPM:{rpm}",       (20, yo), font, 0.6, (0, 255, 255),   2); yo += lh
    cv2.putText(frame, f"FPS:{fps:.1f}",   (20, yo), font, 0.6, (255, 100, 255), 2); yo += lh

    obs_color = (0, 0, 255) if obstacle_detected else (0, 255, 0)
    cv2.putText(frame, f"Obstacle: {'YES' if obstacle_detected else 'NO'}",
                (20, yo), font, 0.6, obs_color, 2); yo += lh
    cv2.putText(frame, f"L-Lane: {'FREE' if left_free else 'BLOCKED'}",
                (20, yo), font, 0.5, (0, 255, 0) if left_free else (0, 0, 255), 2); yo += lh
    cv2.putText(frame, f"R-Lane: {'FREE' if right_free else 'BLOCKED'}",
                (20, yo), font, 0.5, (0, 255, 0) if right_free else (0, 0, 255), 2); yo += lh

    left_det  = "Y" if lane_info.left_line  else "N"
    right_det = "Y" if lane_info.right_line else "N"
    cv2.putText(frame, f"Lines: L[{left_det}] R[{right_det}]",
                (20, yo), font, 0.5, (255, 255, 255), 2); yo += lh

    # ── IMU overlay ──────────────────────────────────────────────────
    cv2.putText(frame, "--- IMU ---",
                (20, yo), font, 0.5, (180, 255, 180), 2); yo += lh
    cv2.putText(frame, f"Roll :{imu_roll:+6.1f} deg",
                (20, yo), font, 0.5, (180, 255, 180), 2); yo += lh
    cv2.putText(frame, f"Pitch:{imu_pitch:+6.1f} deg",
                (20, yo), font, 0.5, (180, 255, 180), 2); yo += lh
    cv2.putText(frame, f"Yaw  :{imu_yaw:+7.1f} deg",
                (20, yo), font, 0.5, (180, 255, 180), 2); yo += lh
    cv2.putText(frame, f"YawRt:{imu_yaw_rate:+6.1f} d/s",
                (20, yo), font, 0.5, (180, 255, 180), 2)

    return frame


COCO_NAMES = {
    0: "person", 1: "bicycle", 2: "car",
    3: "moto",   5: "bus",     7: "truck"
}

def draw_tracks(frame, tracks, fcw_flags):
    """
    Draw tracked objects with ID, class, distance, speed, and FCW status.
    fcw_flags: dict {track_id: bool}
    """
    for track in tracks:
        if track.lost > 0:
            continue   # only draw currently matched tracks
        x1, y1, x2, y2 = track.bbox
        fcw = fcw_flags.get(track.id, False)

        # Box color: red if FCW, orange if in tracking, green otherwise
        color = (0, 0, 255) if fcw else (0, 165, 255)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # Build label lines
        label_name  = f"ID{track.id} {COCO_NAMES.get(track.cls, track.cls)}"
        label_dist  = f"{track.dist_m:.1f}m"  if hasattr(track, 'dist_m')  else ""
        label_speed = f"{track.speed_ms:.1f}m/s" if hasattr(track, 'speed_ms') else ""
        label_fcw   = " !! FCW !!" if fcw else ""

        full_label = f"{label_name} | {label_dist} | {label_speed}{label_fcw}"
        cv2.putText(frame, full_label, (x1, max(12, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)

        # Draw centroid dot
        cx, cy = track.centroid
        cv2.circle(frame, (cx, cy), 4, color, -1)

        # FCW flash border
        if fcw:
            cv2.rectangle(frame, (x1 - 3, y1 - 3), (x2 + 3, y2 + 3), (0, 0, 255), 3)

    # Global FCW banner
    if any(fcw_flags.values()):
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 40), (0, 0, 200), -1)
        cv2.putText(frame, "!! FORWARD COLLISION WARNING !!",
                    (60, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    return frame


# =====================================
# MAIN
# =====================================
def main():
    global current_state, target_lane_center, lane_change_cooldown_counter

    # ── Initialize IMU ───────────────────────────────────────────────
    imu = IMU(bus, IMU_ADDR)
    imu.begin()
    imu.calibrate()          # blocks ~2 s with CALIB_SAMPLES=200 @ 10 ms each

    # ── Initialize camera ────────────────────────────────────────────
    picam2 = Picamera2()
    cfg = picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        controls={"FrameRate": 30}
    )
    picam2.configure(cfg)
    picam2.start()
    time.sleep(2)

    print("Enhanced Lane Detection with IMU + Obstacle Avoidance Started")

    lane_info         = LaneInfo()
    obstacle_detector = ObstacleDetector(WIDTH, HEIGHT)
    occupancy_checker = LaneOccupancyChecker(WIDTH)
    decision_maker    = LaneChangeDecisionMaker()
    lane_changer      = SmoothLaneChanger(WIDTH)
    tracker           = CentroidTracker()
    dist_estimator    = DistanceEstimator()
    speed_estimator   = SpeedEstimator()
    fcw_system        = FCWSystem()

    last_lane_center = WIDTH // 2
    frame_count = 0
    start_time  = time.time()
    fps = 0.0

    try:
        while True:
            # ── 1. IMU UPDATE ─────────────────────────────────────────
            imu.update()
            imu_roll, imu_pitch, imu_yaw = imu.angles
            imu_yaw_rate = imu.yaw_rate

            # ── 2. CAPTURE FRAME ──────────────────────────────────────
            frame = picam2.capture_array()
            frame = cv2.flip(frame, 0)

            frame_count += 1
            if frame_count % 10 == 0:
                fps        = 10 / (time.time() - start_time)
                start_time = time.time()

            h, w = frame.shape[:2]
            car_center = w // 2

            # ── 3. LANE DETECTION ─────────────────────────────────────
            edges = preprocess(frame)
            if DEBUG_WINDOWS:
                cv2.imshow("Edges", edges)

            left_lane, right_lane = detect_lane_lines(edges, frame.shape)
            lane_info.update(left_lane, right_lane, w)

            # 6. EMA temporal smoothing for lane centre
            raw_center = lane_info.current_lane_center
            if last_lane_center == WIDTH // 2:
                # first frame — seed EMA with raw value, no smoothing
                smoothed_lane_center = raw_center
            else:
                smoothed_lane_center = int(
                    EMA_ALPHA * last_lane_center + (1.0 - EMA_ALPHA) * raw_center
                )
            last_lane_center = smoothed_lane_center

            # ── 4. OBSTACLE DETECTION ─────────────────────────────────
            obstacle_detected, detections = obstacle_detector.detect_obstacles(frame)

            # ── 4a. TRACKING ──────────────────────────────────────────
            tracks = tracker.update(detections)

            # ── 4b. DISTANCE + SPEED + FCW per track ──────────────────
            now_t    = time.time()
            fcw_flags = {}
            any_fcw   = False
            for track in tracks:
                if track.lost > 0:
                    continue
                # Distance
                dist_m          = dist_estimator.estimate(track.bbox, track.cls)
                track.dist_m    = dist_m
                # Speed
                speed_ms        = speed_estimator.update(track, dist_m, now_t)
                track.speed_ms  = speed_ms
                # FCW
                in_dz           = obstacle_detector.is_in_danger_zone(track.bbox)
                fcw             = fcw_system.update(track, dist_m, in_dz)
                fcw_flags[track.id] = fcw
                if fcw:
                    any_fcw = True

            # If FCW triggered, treat as obstacle regardless of YOLO danger zone
            if any_fcw:
                obstacle_detected = True

            # ── 5. LANE OCCUPANCY ─────────────────────────────────────
            left_lane_free, right_lane_free = occupancy_checker.check_lane_occupancy(
                detections, lane_info)

            # ── 6. FSM DECISION ───────────────────────────────────────
            new_state, should_manage_cooldown = decision_maker.make_decision(
                current_state, obstacle_detected, left_lane_free,
                right_lane_free, lane_change_cooldown_counter)

            if new_state != current_state:
                print(f"State: {current_state.name} → {new_state.name}")
                current_state = new_state
                if new_state in (DrivingState.CHANGE_LEFT, DrivingState.CHANGE_RIGHT):
                    lane_change_cooldown_counter = LANE_CHANGE_COOLDOWN

            # ── 7. SMOOTH LANE CHANGE ─────────────────────────────────
            target_center, lane_change_complete = lane_changer.update_target(
                current_state, lane_info, smoothed_lane_center)

            if lane_change_complete:
                print("Lane change complete → KEEP_LANE")
                current_state = DrivingState.KEEP_LANE
                lane_change_cooldown_counter = 0

            if lane_change_cooldown_counter > 0:
                lane_change_cooldown_counter -= 1

            # ── 8. PD + IMU STEERING ──────────────────────────────────
            global prev_raw_error, spike_active
            raw_error   = target_center - car_center
            error_delta = abs(raw_error - prev_raw_error)

            if error_delta > SPIKE_THRESHOLD_PX:
                # Sudden large error — check opposite lane instead of
                # steering hard toward the potentially false detection.
                y_ref = int(frame.shape[0] * 0.8)

                if raw_error > 0:
                    # Target jumped right → left lane may be the false one.
                    # Re-estimate center from the RIGHT lane.
                    rx = get_lane_position(lane_info.right_line, y_ref)
                    if rx is not None:
                        raw_error = car_center - (rx - lane_info.lane_width // 2)
                else:
                    # Target jumped left → right lane may be the false one.
                    # Re-estimate center from the LEFT lane.
                    lx = get_lane_position(lane_info.left_line, y_ref)
                    if lx is not None:
                        raw_error = car_center - (lx + lane_info.lane_width // 2)

                spike_active = True
            else:
                spike_active = False

            prev_raw_error = raw_error
            error_buffer.append(raw_error)
            smooth_error = int(np.mean(error_buffer))

            steer = compute_steer(smooth_error, yaw_rate=imu_yaw_rate)

            # ── 9. SPEED CONTROL ──────────────────────────────────────
            rpm = compute_dynamic_rpm(abs(steer), current_state)
            if abs(smooth_error) > 60:
                rpm = MIN_RPM

            # ── 10. ACTUATION ─────────────────────────────────────────
            desired_angle = int(98 + steer)  # 0°=right, 180°=left, center=98°
            actual_angle  = desired_angle
            send_data(rpm, desired_angle, actual_angle)

            # ── 11. VISUALIZATION ─────────────────────────────────────
            frame = draw_comprehensive_info(
                frame, lane_info, target_center, smooth_error,
                desired_angle, rpm, fps, current_state,
                obstacle_detected, left_lane_free, right_lane_free,
                lane_changer.stabilization_counter,
                imu_roll, imu_pitch, imu_yaw, imu_yaw_rate)

            frame = draw_tracks(frame, tracks, fcw_flags)
            obstacle_detector.draw_danger_zone(frame)

            cv2.imshow("Enhanced ADAS with IMU", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nShutdown requested")

    finally:
        print("Cleaning up…")
        send_data(0, 98, 98)
        picam2.stop()
        bus.close()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
