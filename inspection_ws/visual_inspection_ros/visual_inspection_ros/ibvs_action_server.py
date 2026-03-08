#!/usr/bin/env python3
"""
ibvs_action_server.py -- ROS2 Action Server: Visual inspection pipeline.

Pipeline:
  Stage 1  DETECT (Insta360 only, up to 20s):
    - YOLO on Insta360 → detect object
    - degree-4 calibration formula → coarse servo angles
    - pan-tilt moves → stop Insta360 YOLO

  Stage 2  IBVS (Logitech only, time-based 40s total):
    - YOLO on Logitech → PID servo loop until err < 10px
    - Object can disappear for up to 3s (noise/occlusion) without aborting
    - After 40s total, abort with ibvs_timeout

  Capture
    - 4 images from Logitech once centred
    - Publish via MQTT

Debug (view in RViz2):
  /visual_inspection/debug   -- combined side-by-side view with arrow, mode, FPS
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge

from visual_inspection_interfaces.action import InspectObjects

import cv2
import numpy as np
import time
import os
import threading


# ---------------------------------------------------------------------------
# Calibration -- degree-4 polynomial (from calibration_config.py)
# ---------------------------------------------------------------------------

def calculate_pan(x, y):
    val = 155.6668858701
    val += 0.4560199871 * x + 0.7342655133 * y
    val += -0.0035976159 * (x**2) + -0.0043827793 * (x*y) + -0.0084661672 * (y**2)
    val += 0.0000057403 * (x**3) + 0.0000133400 * (x**2*y) + 0.0000101783 * (x*y**2) + 0.0000536484 * (y**3)
    val += -0.0000000029*(x**4) + -0.0000000106*(x**3*y) + -0.0000000180*(x**2*y**2) + 0.0000000082*(x*y**3) + -0.0000001569*(y**4)
    return float(np.clip(val, 0, 180))


def calculate_tilt(x, y):
    val = 113.6717028380
    val += 0.3335993613 * x + -0.6916488737 * y
    val += -0.0008407415 * (x**2) + -0.0009024326 * (x*y) + 0.0084145572 * (y**2)
    val += 0.0000005454 * (x**3) + 0.0000028613 * (x**2*y) + 0.0000022977 * (x*y**2) + -0.0000683027 * (y**3)
    val += 0.0000000001*(x**4) + -0.0000000038*(x**3*y) + 0.0000000088*(x**2*y**2) + -0.0000000351*(x*y**3) + 0.0000002091*(y**4)
    return float(np.clip(val, 20, 160))


def clamp(val, lo=0, hi=180):
    return int(max(lo, min(hi, round(val))))


# ---------------------------------------------------------------------------
# YOLO loader
# ---------------------------------------------------------------------------

def load_yolo(engine_path):
    try:
        from ultralytics import YOLO
        if os.path.exists(engine_path):
            model = YOLO(engine_path)
            print(f'[ibvs_action_server] YOLO TensorRT: {engine_path}')
        else:
            pt = engine_path.replace('.engine', '.pt')
            model = YOLO(pt)
            print(f'[ibvs_action_server] YOLO PT fallback: {pt}')
        return model
    except Exception as e:
        print(f'[ibvs_action_server] YOLO load failed: {e}')
        return None


# ---------------------------------------------------------------------------
# Action Server
# ---------------------------------------------------------------------------

class IBVSActionServer(Node):

    # Paths
    ENGINE_PATH = os.path.expanduser('~/Documents/Visual_Inspection_ws/weights/yolo11n.engine')

    # YOLO confidence
    CONF_INSTA  = 0.5   # coarse detection
    CONF_IBVS   = 0.3   # IBVS (object may be partial/close)

    # Timeouts
    INSTA_SEARCH_TIMEOUT  = 20.0  # max wait for initial detection on Insta360
    LOGI_FIRST_DET_WAIT   = 5.0   # wait for first detection in Logitech after coarse
    IBVS_TOTAL_TIMEOUT    = 40.0  # total IBVS time budget (seconds)
    IBVS_LOST_PATIENCE    = 3.0   # how long object can vanish during IBVS

    # Timing
    COARSE_WAIT  = 2.0   # seconds for servo to reach coarse position

    # IBVS PID (KP/KI/KD from ibvs_pipeline.py)
    KP = 0.12;  KI = 0.002;  KD = 0.02
    MAX_STEP_DEG = 3.0
    SLOW_ZONE_PX = 15.0
    DEADBAND_DEG = 0.5
    IBVS_TOL_PX  = 10.0

    # Logitech frame info (640x480)
    CX_LOGI = 320.0;  CY_LOGI = 240.0
    FX_LOGI = 640.0;  FY_LOGI = 640.0

    # Capture
    IMAGES_PER_OBJ = 4
    CAPTURE_DELAY  = 0.5

    # Tilt servo mounted in reverse -- flip to 180-tilt
    TILT_REVERSED  = True

    # Debug frame size
    DEBUG_W = 640;  DEBUG_H = 360  # each camera panel

    def __init__(self):
        super().__init__('ibvs_action_server')
        self.bridge   = CvBridge()
        self.cb_group = ReentrantCallbackGroup()

        # Latest frames
        self._lock_insta  = threading.Lock()
        self._lock_logi   = threading.Lock()
        self._frame_insta = None
        self._frame_logi  = None

        # Subscribers
        self.create_subscription(Image, '/visual_inspection/insta360/image_raw',
                                 self._cb_insta, 10, callback_group=self.cb_group)
        self.create_subscription(Image, '/visual_inspection/logitech/image_raw',
                                 self._cb_logi,  10, callback_group=self.cb_group)

        # Publishers
        self.servo_pub = self.create_publisher(Int16MultiArray, '/servo/pan_tilt', 10)
        self.debug_pub = self.create_publisher(Image, '/visual_inspection/debug', 10)

        # YOLO
        self.get_logger().info('Loading YOLO model...')
        self.model = load_yolo(self.ENGINE_PATH)
        if self.model:
            self.get_logger().info('YOLO model ready')
        else:
            self.get_logger().error('YOLO load failed')

        # State visible to debug timer
        self._yolo_lock    = threading.Lock()
        self._mode         = 'IDLE'
        self._ibvs_ex      = 0.0
        self._ibvs_ey      = 0.0
        self._ibvs_iter    = 0
        self._ibvs_err     = 0.0
        self._fps_counter  = 0
        self._fps_time     = time.time()
        self._fps          = 0.0
        self._goal_active  = False

        # Action server
        self._action_server = ActionServer(
            self, InspectObjects, '/visual_inspection/inspect_objects',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        # Debug timer (4Hz)
        self.create_timer(0.25, self._debug_timer_cb)

        self.get_logger().info('Action server ready at /visual_inspection/inspect_objects')
        self.get_logger().info('Debug topic: /visual_inspection/debug  (add Image in RViz2)')

    # ---- Camera callbacks ---------------------------------------------------

    def _cb_insta(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lock_insta:
            self._frame_insta = frame

    def _cb_logi(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lock_logi:
            self._frame_logi = frame

    def _get_insta(self):
        with self._lock_insta:
            return self._frame_insta.copy() if self._frame_insta is not None else None

    def _get_logi(self):
        with self._lock_logi:
            return self._frame_logi.copy() if self._frame_logi is not None else None

    # ---- Goal / cancel ------------------------------------------------------

    def goal_callback(self, goal_request):
        self.get_logger().info('Goal received')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    # ---- Servo helpers -------------------------------------------------------

    def _servo(self, tilt, pan):
        """Send servo command. Tilt is flipped if TILT_REVERSED."""
        t = 180 - clamp(tilt) if self.TILT_REVERSED else clamp(tilt)
        msg = Int16MultiArray()
        msg.data = [t, clamp(pan)]
        self.servo_pub.publish(msg)

    def _home(self):
        self._servo(90, 90)
        self.get_logger().info('Servos home (90, 90)')

    # ---- YOLO detection -----------------------------------------------------

    def _detect_raw(self, frame, conf):
        """Run YOLO. Returns (detections, annotated_frame). Call with lock held."""
        if self.model is None or frame is None:
            return [], frame.copy() if frame is not None else None

        results = self.model(frame, verbose=False, conf=conf)[0]
        debug   = frame.copy()
        dets    = []

        for box in results.boxes:
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            c  = float(box.conf[0])
            dets.append((cx, cy, x1, y1, x2, y2, c))
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(debug, f'{c:.2f}', (x1, max(y1-5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        dets.sort(key=lambda d: d[6], reverse=True)
        h, w = frame.shape[:2]
        cv2.line(debug, (w//2, 0), (w//2, h), (80, 80, 200), 1)
        cv2.line(debug, (0, h//2), (w, h//2), (80, 80, 200), 1)

        if dets:
            cx, cy = dets[0][0], dets[0][1]
            cv2.circle(debug, (int(cx), int(cy)), 6, (0, 0, 255), -1)

        return dets, debug

    def _detect_insta(self, conf=None):
        """Detect on Insta360. Returns (dets, raw_frame, annotated_frame)."""
        frame = self._get_insta()
        if frame is None:
            return [], None, None
        c = conf or self.CONF_INSTA
        with self._yolo_lock:
            dets, dbg = self._detect_raw(frame, c)
        return dets, frame, dbg

    def _detect_logi(self, conf=None):
        """Detect on Logitech. Returns (dets, raw_frame, annotated_frame)."""
        frame = self._get_logi()
        if frame is None:
            return [], None, None
        c = conf or self.CONF_IBVS
        with self._yolo_lock:
            dets, dbg = self._detect_raw(frame, c)
            # Draw IBVS arrow if detection found
            if dets:
                h, w = frame.shape[:2]
                cx_d, cy_d = int(dets[0][0]), int(dets[0][1])
                cx_f, cy_f = w // 2, h // 2
                cv2.arrowedLine(dbg, (cx_d, cy_d), (cx_f, cy_f), (255, 0, 0), 2)
        return dets, frame, dbg

    # ---- Combined debug publisher -------------------------------------------

    def _build_debug_frame(self, insta_dbg, logi_dbg):
        """Build combined side-by-side debug image (1280 x 360)."""
        W, H = self.DEBUG_W, self.DEBUG_H

        # Left: Insta360
        if insta_dbg is not None:
            left = cv2.resize(insta_dbg, (W, H))
        else:
            left = np.zeros((H, W, 3), dtype=np.uint8)
        cv2.putText(left, 'INSTA360 (Coarse)', (10, H-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Right: Logitech
        if logi_dbg is not None:
            right = cv2.resize(logi_dbg, (W, H))
        else:
            right = np.zeros((H, W, 3), dtype=np.uint8)
        cv2.putText(right, 'LOGITECH (IBVS)', (10, H-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # IBVS overlay on right
        if self._mode == 'IBVS':
            cv2.putText(right,
                        f'IBVS: err=({self._ibvs_ex:.1f},{self._ibvs_ey:.1f})  '
                        f'Iter={self._ibvs_iter}  px={self._ibvs_err:.1f}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        combined = np.hstack([left, right])

        # Mode + FPS top-left
        cv2.putText(combined, f'Mode: {self._mode}   FPS: {self._fps:.1f}',
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return combined

    def _publish_debug(self, insta_dbg, logi_dbg):
        frame = self._build_debug_frame(insta_dbg, logi_dbg)
        msg   = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_pub.publish(msg)

        # FPS counter
        self._fps_counter += 1
        now = time.time()
        if now - self._fps_time >= 2.0:
            self._fps      = self._fps_counter / (now - self._fps_time)
            self._fps_counter = 0
            self._fps_time    = now

    def _debug_timer_cb(self):
        """4Hz timer — publishes combined debug when no active goal."""
        if self._goal_active:
            return
        if not self._yolo_lock.acquire(blocking=False):
            return
        try:
            insta_f = self._get_insta()
            logi_f  = self._get_logi()
            _, insta_dbg = self._detect_raw(insta_f, self.CONF_INSTA) if insta_f is not None else ([], None)
            _, logi_dbg  = self._detect_raw(logi_f,  self.CONF_INSTA) if logi_f  is not None else ([], None)
        finally:
            self._yolo_lock.release()

        self._publish_debug(insta_dbg, logi_dbg)

    # ---- Stage 1: search Insta360 -------------------------------------------

    def _search_insta(self, goal_handle):
        """Search Insta360 up to 20s. Returns first detection (cx, cy, conf) or None."""
        self._mode = 'DETECTING'
        self.get_logger().info(f'  Searching Insta360 (up to {self.INSTA_SEARCH_TIMEOUT}s)...')
        deadline = time.time() + self.INSTA_SEARCH_TIMEOUT

        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                return None

            dets, _, insta_dbg = self._detect_insta()
            logi_dbg = None
            logi_f = self._get_logi()
            if logi_f is not None:
                logi_dbg = logi_f.copy()
            self._publish_debug(insta_dbg, logi_dbg)

            if dets:
                cx, cy, _, _, _, _, conf = dets[0]
                self.get_logger().info(f'  Object found on Insta360: cx={cx:.0f} cy={cy:.0f} conf={conf:.2f}')
                return cx, cy, conf

            time.sleep(0.1)

        self.get_logger().warn('  No object found on Insta360 within timeout')
        return None

    # ---- Stage 2: IBVS on Logitech ------------------------------------------

    def _ibvs(self, goal_handle, start_pan, start_tilt, feedback, obj_idx):
        """IBVS on Logitech. Time-based 40s budget. Object can vanish 3s."""
        self._mode = 'IBVS'
        self.get_logger().info(f'  IBVS starting from pan={start_pan:.1f} tilt={start_tilt:.1f}')

        # PID state
        integral_pan = integral_tilt = 0.0
        prev_ep = prev_et = 0.0
        pan  = float(start_pan)
        tilt = float(start_tilt)
        dt   = 0.066  # ~15Hz

        last_seen_time = time.time()
        start_time     = time.time()
        ibvs_iter      = 0
        insta_dbg_cache = None  # freeze Insta360 side during IBVS

        # Cache last Insta360 debug frame
        _, _, insta_dbg_cache = self._detect_insta()

        deadline = start_time + self.IBVS_TOTAL_TIMEOUT

        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                return False

            dets, _, logi_dbg = self._detect_logi()
            self._publish_debug(insta_dbg_cache, logi_dbg)

            now = time.time()

            if not dets:
                if now - last_seen_time > self.IBVS_LOST_PATIENCE:
                    self.get_logger().warn(f'  IBVS: object not seen for {now-last_seen_time:.1f}s -- aborting')
                    return False
                self.get_logger().info(f'  IBVS: object not seen ({now-last_seen_time:.1f}s) -- waiting...')
                time.sleep(dt)
                continue

            last_seen_time = now
            cx_d, cy_d = dets[0][0], dets[0][1]
            ex  = cx_d - self.CX_LOGI
            ey  = cy_d - self.CY_LOGI
            err = (ex**2 + ey**2) ** 0.5

            # Update overlay state
            self._ibvs_ex   = ex
            self._ibvs_ey   = ey
            self._ibvs_iter = ibvs_iter
            self._ibvs_err  = err

            feedback.current_step   = 'ibvs'
            feedback.current_object = obj_idx
            feedback.ibvs_error_px  = float(err)
            goal_handle.publish_feedback(feedback)

            if err < self.IBVS_TOL_PX:
                self.get_logger().info(f'  IBVS converged: err={err:.1f}px at iter {ibvs_iter}')
                return True

            # Angular errors
            theta_x = np.degrees(np.arctan(ex / self.FX_LOGI))
            theta_y = np.degrees(np.arctan(ey / self.FY_LOGI))

            if abs(theta_x) < self.DEADBAND_DEG: theta_x = 0.0
            if abs(theta_y) < self.DEADBAND_DEG: theta_y = 0.0

            if abs(ex) > self.SLOW_ZONE_PX:
                theta_y = 0.0
                integral_tilt = 0.0

            # PID pan
            integral_pan = np.clip(integral_pan + theta_x * dt, -50, 50)
            dp = -(self.KP*theta_x + self.KI*integral_pan + self.KD*(theta_x-prev_ep)/dt)
            prev_ep = theta_x

            # PID tilt
            integral_tilt = np.clip(integral_tilt + theta_y * dt, -50, 50)
            dt_ = -(self.KP*theta_y + self.KI*integral_tilt + self.KD*(theta_y-prev_et)/dt)
            prev_et = theta_y

            # Velocity limit
            sp = min(1.0, abs(ex)/self.SLOW_ZONE_PX)
            st = min(1.0, abs(ey)/self.SLOW_ZONE_PX)
            dp  = np.clip(dp,  -self.MAX_STEP_DEG*sp,  self.MAX_STEP_DEG*sp)
            dt_ = np.clip(dt_, -self.MAX_STEP_DEG*st, self.MAX_STEP_DEG*st)

            pan  = float(np.clip(pan  + dp,  0,  180))
            tilt = float(np.clip(tilt + dt_, 20, 160))
            self._servo(tilt, pan)

            if ibvs_iter % 10 == 0:
                self.get_logger().info(f'  IBVS iter {ibvs_iter}: err={err:.1f}px pan={pan:.1f} tilt={tilt:.1f}')

            ibvs_iter += 1
            time.sleep(dt)

        self.get_logger().warn(f'  IBVS timeout after {self.IBVS_TOTAL_TIMEOUT}s')
        return False

    # ---- Image capture -------------------------------------------------------

    def _capture(self, n=4):
        imgs = []
        for i in range(n):
            f = self._get_logi()
            if f is not None:
                imgs.append(f.copy())
                self.get_logger().info(f'  Captured {i+1}/{n}')
            time.sleep(self.CAPTURE_DELAY)
        return imgs

    # ---- MQTT ----------------------------------------------------------------

    def _mqtt(self, images, obj_id):
        try:
            import paho.mqtt.client as mqtt, base64, json
            client = mqtt.Client()
            client.connect('localhost', 1883, timeout=5)
            for i, img in enumerate(images):
                _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                client.publish(f'visual_inspection/object_{obj_id}/images',
                               json.dumps({'object_id': obj_id, 'image_idx': i,
                                           'timestamp': time.time(),
                                           'image_b64': base64.b64encode(buf.tobytes()).decode()}))
            client.disconnect()
            self.get_logger().info(f'  MQTT: {len(images)} images published')
        except Exception as e:
            self.get_logger().warn(f'  MQTT skipped: {e}')

    # ---- Main execute --------------------------------------------------------

    def execute_callback(self, goal_handle):
        self.get_logger().info('Inspection started')
        self._goal_active = True

        feedback = InspectObjects.Feedback()
        result   = InspectObjects.Result()
        max_obj  = goal_handle.request.max_objects
        ret_home = goal_handle.request.return_home

        def abort(reason):
            self._mode        = 'IDLE'
            self._goal_active = False
            result.success           = False
            result.objects_inspected = 0
            result.failed_reason     = reason
            if ret_home:
                self._home()
            goal_handle.abort()
            return result

        # ---- Stage 1: detect on Insta360 ----
        found = self._search_insta(goal_handle)
        if found is None:
            return abort('no_detection')

        cx_obj, cy_obj, conf = found

        # ---- Coarse positioning ----
        self._mode = 'COARSE'
        pan_c  = calculate_pan(cx_obj, cy_obj)
        tilt_c = calculate_tilt(cx_obj, cy_obj)
        self._servo(tilt_c, pan_c)
        self.get_logger().info(f'  Coarse move: pan={pan_c:.1f} tilt={tilt_c:.1f} -- waiting {self.COARSE_WAIT}s')

        # Show debug while waiting for servo
        t_wait = time.time()
        while time.time() - t_wait < self.COARSE_WAIT:
            _, _, insta_dbg = self._detect_insta()
            logi_f = self._get_logi()
            self._publish_debug(insta_dbg, logi_f)
            time.sleep(0.1)

        # ---- Stage 2: Wait for first detection in Logitech ----
        self.get_logger().info(f'  Waiting up to {self.LOGI_FIRST_DET_WAIT}s for object in Logitech...')
        self._mode = 'IBVS'
        deadline   = time.time() + self.LOGI_FIRST_DET_WAIT
        first_det  = False
        _, _, insta_dbg_frozen = self._detect_insta()

        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                return abort('cancelled')
            dets, _, logi_dbg = self._detect_logi()
            self._publish_debug(insta_dbg_frozen, logi_dbg)
            if dets:
                first_det = True
                self.get_logger().info('  Object visible in Logitech -- starting IBVS')
                break
            time.sleep(0.1)

        if not first_det:
            self.get_logger().warn('  Object not visible in Logitech -- coarse calibration may need retuning')
            return abort('logi_no_detection')

        # ---- IBVS ----
        centred = self._ibvs(goal_handle, pan_c, tilt_c, feedback, 1)

        if not centred:
            return abort('ibvs_timeout')

        # ---- Capture ----
        self._mode = 'CAPTURING'
        self.get_logger().info('  Capturing images...')
        imgs = self._capture(self.IMAGES_PER_OBJ)
        self._mqtt(imgs, 1)

        # ---- Done ----
        self._mode        = 'IDLE'
        self._goal_active = False
        if ret_home:
            self._home()

        result.success           = True
        result.objects_inspected = 1
        result.failed_reason     = ''
        goal_handle.succeed()
        self.get_logger().info('Inspection complete')
        return result


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = IBVSActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
