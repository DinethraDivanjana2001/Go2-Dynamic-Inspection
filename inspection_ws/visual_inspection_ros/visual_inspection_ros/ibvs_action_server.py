#!/usr/bin/env python3
"""
ibvs_action_server.py -- ROS2 Action Server: Visual inspection pipeline.

Exposes action: /visual_inspection/inspect_objects (InspectObjects.action)
BehaviorTree.CPP calls this via BT::RosActionNode<>.

Pipeline for each goal:
  1. Detect objects on Insta360 frame (YOLO TensorRT)
  2. For each detected object:
     a. Coarse positioning (cubic formula -> /servo/pan_tilt)
     b. IBVS centering (pixel error loop -> /servo/pan_tilt)
     c. Capture 4 images from Logitech
     d. Publish images via MQTT
  3. Return servos to home (90,90)

Subscribes:
  /visual_inspection/insta360/image_raw  (sensor_msgs/Image)
  /visual_inspection/logitech/image_raw  (sensor_msgs/Image)

Publishes:
  /servo/pan_tilt  (std_msgs/Int16MultiArray) -- servo_node receives this

Action:
  /visual_inspection/inspect_objects  (visual_inspection_interfaces/InspectObjects)
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
import sys
import threading


# ---------------------------------------------------------------------------
# Calibration -- coarse positioning (linear, fitted from calibration_points.csv)
# Pan  = PAN_M  * pixel_x + PAN_C
# Tilt = TILT_M * pixel_y + TILT_C
# Fitted using 156 calibration points. RMSE: pan=5.45 deg, tilt=5.58 deg.
# Run tools/calib_fov.py to recalibrate if mount/camera changes.
# ---------------------------------------------------------------------------

PAN_M  = -0.33851
PAN_C  =  205.84
TILT_M = -0.32650
TILT_C =  139.99

# Frame centre for 640x360 Insta360 display
CX = 320.0
CY = 180.0


def coarse_pan(px):
    """Map Insta360 pixel column to pan servo angle."""
    return float(np.clip(PAN_M * px + PAN_C, 0, 180))


def coarse_tilt(py):
    """Map Insta360 pixel row to tilt servo angle."""
    return float(np.clip(TILT_M * py + TILT_C, 0, 180))


def clamp_servo(val):
    return int(max(0, min(180, round(val))))


# ---------------------------------------------------------------------------
# YOLO loader
# ---------------------------------------------------------------------------

def load_yolo(engine_path):
    """Load YOLO model (TensorRT engine or fallback to PyTorch weights)."""
    try:
        from ultralytics import YOLO
        if os.path.exists(engine_path):
            model = YOLO(engine_path)
            print(f'[ibvs_action_server] YOLO loaded from TensorRT engine: {engine_path}')
        else:
            # Fallback to pytorch weights
            pt_path = engine_path.replace('.engine', '.pt')
            model = YOLO(pt_path)
            print(f'[ibvs_action_server] WARNING: engine not found, using PT weights: {pt_path}')
        return model
    except Exception as e:
        print(f'[ibvs_action_server] YOLO load failed: {e}')
        return None


# ---------------------------------------------------------------------------
# Action Server Node
# ---------------------------------------------------------------------------

class IBVSActionServer(Node):

    # ---- Config (set these to match your system) ----------------------------
    ENGINE_PATH   = os.path.expanduser('~/Documents/Visual_Inspection_ws/weights/yolo11n.engine')
    IBVS_MAX_ITER = 150      # max IBVS iterations before timeout
    IBVS_GAIN     = 0.3      # IBVS proportional gain
    IBVS_TOL_PX   = 15.0    # pixel error tolerance to declare centred
    IMAGES_PER_OBJ = 4       # number of Logitech images to capture per object
    RECENTER_WAIT = 0.5      # seconds to wait after repositioning before capture
    # -------------------------------------------------------------------------

    def __init__(self):
        super().__init__('ibvs_action_server')
        self.bridge   = CvBridge()
        self.cb_group = ReentrantCallbackGroup()

        # Latest frames from camera_node (thread-safe)
        self._lock_insta = threading.Lock()
        self._lock_logi  = threading.Lock()
        self._frame_insta = None
        self._frame_logi  = None

        # Subscribers
        self.create_subscription(
            Image,
            '/visual_inspection/insta360/image_raw',
            self._cb_insta,
            10,
            callback_group=self.cb_group
        )
        self.create_subscription(
            Image,
            '/visual_inspection/logitech/image_raw',
            self._cb_logi,
            10,
            callback_group=self.cb_group
        )

        # Servo publisher
        self.servo_pub = self.create_publisher(Int16MultiArray, '/servo/pan_tilt', 10)

        # YOLO model
        self.get_logger().info('Loading YOLO model...')
        self.model = load_yolo(self.ENGINE_PATH)
        if self.model:
            self.get_logger().info('YOLO model ready')
        else:
            self.get_logger().error('YOLO model failed to load -- detection will not work')

        # Action server
        self._action_server = ActionServer(
            self,
            InspectObjects,
            '/visual_inspection/inspect_objects',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('Action server ready at /visual_inspection/inspect_objects')

    # ---- Camera subscribers --------------------------------------------------

    def _cb_insta(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lock_insta:
            self._frame_insta = frame

    def _cb_logi(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lock_logi:
            self._frame_logi = frame

    def _get_insta_frame(self):
        with self._lock_insta:
            return self._frame_insta.copy() if self._frame_insta is not None else None

    def _get_logi_frame(self):
        with self._lock_logi:
            return self._frame_logi.copy() if self._frame_logi is not None else None

    # ---- Goal / cancel callbacks --------------------------------------------

    def goal_callback(self, goal_request):
        self.get_logger().info('Inspection goal received')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Inspection cancel requested')
        return CancelResponse.ACCEPT

    # ---- Servo helpers -------------------------------------------------------

    def _send_servo(self, tilt, pan):
        msg = Int16MultiArray()
        msg.data = [clamp_servo(tilt), clamp_servo(pan)]
        self.servo_pub.publish(msg)

    def _home_servos(self):
        self._send_servo(90, 90)
        self.get_logger().info('Servos returned to home (90, 90)')

    # ---- Detection -----------------------------------------------------------

    def _detect_objects(self, frame):
        """Run YOLO on frame. Returns list of (cx, cy, confidence, class_id)."""
        if self.model is None or frame is None:
            return []
        results = self.model(frame, verbose=False)[0]
        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx   = (x1 + x2) / 2.0
            cy   = (y1 + y2) / 2.0
            conf = float(box.conf[0])
            cls  = int(box.cls[0])
            detections.append((cx, cy, conf, cls))
        # Sort by confidence descending
        detections.sort(key=lambda d: d[2], reverse=True)
        return detections

    # ---- IBVS ----------------------------------------------------------------

    def _ibvs_center(self, goal_handle, obj_idx, feedback):
        """Run IBVS loop until object centred or timeout. Returns True if centred."""
        self.get_logger().info(f'  IBVS centering object {obj_idx}...')
        tilt = 90.0
        pan  = 90.0

        for i in range(self.IBVS_MAX_ITER):
            if goal_handle.is_cancel_requested:
                return False

            frame = self._get_insta_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            dets = self._detect_objects(frame)
            if not dets:
                self.get_logger().warn(f'  IBVS: lost object at iteration {i}')
                time.sleep(0.05)
                continue

            cx_det, cy_det = dets[0][0], dets[0][1]
            ex = cx_det - CX
            ey = cy_det - CY
            err = (ex**2 + ey**2) ** 0.5

            # Publish feedback
            feedback.current_step   = 'ibvs'
            feedback.current_object = obj_idx
            feedback.ibvs_error_px  = float(err)
            goal_handle.publish_feedback(feedback)

            if err < self.IBVS_TOL_PX:
                self.get_logger().info(f'  IBVS converged: err={err:.1f}px at iter {i}')
                return True

            # Proportional control
            tilt -= self.IBVS_GAIN * (ey / CY) * 90.0
            pan  += self.IBVS_GAIN * (ex / CX) * 90.0
            self._send_servo(tilt, pan)
            time.sleep(0.05)

        self.get_logger().warn('  IBVS timeout -- max iterations reached')
        return False

    # ---- Image capture -------------------------------------------------------

    def _capture_images(self, n=4):
        """Capture n images from Logitech. Returns list of numpy arrays."""
        images = []
        for i in range(n):
            frame = self._get_logi_frame()
            if frame is not None:
                images.append(frame.copy())
                self.get_logger().info(f'  Captured image {i+1}/{n}')
            time.sleep(self.RECENTER_WAIT)
        return images

    # ---- MQTT publish -------------------------------------------------------

    def _publish_mqtt(self, images, obj_id):
        """Publish captured images to MQTT broker. Returns True on success."""
        try:
            import paho.mqtt.client as mqtt
            import base64
            import json

            # TODO: update broker IP when available
            BROKER = 'localhost'
            PORT   = 1883
            TOPIC  = f'visual_inspection/object_{obj_id}/images'

            client = mqtt.Client()
            client.connect(BROKER, PORT, timeout=5)

            for i, img in enumerate(images):
                _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                payload = json.dumps({
                    'object_id':  obj_id,
                    'image_idx':  i,
                    'timestamp':  time.time(),
                    'image_b64':  base64.b64encode(buf.tobytes()).decode()
                })
                client.publish(TOPIC, payload)

            client.disconnect()
            self.get_logger().info(f'  Published {len(images)} images to MQTT (object {obj_id})')
            return True

        except Exception as e:
            self.get_logger().error(f'  MQTT publish failed: {e}')
            return False

    # ---- Main execute --------------------------------------------------------

    def execute_callback(self, goal_handle):
        self.get_logger().info('Starting inspection...')

        max_objects  = goal_handle.request.max_objects
        return_home  = goal_handle.request.return_home
        feedback     = InspectObjects.Feedback()
        result       = InspectObjects.Result()

        # Step 1: detect objects
        feedback.current_step = 'detecting'
        goal_handle.publish_feedback(feedback)

        frame = self._get_insta_frame()
        if frame is None:
            self.get_logger().error('No camera frame -- is camera_node running?')
            result.success          = False
            result.objects_inspected = 0
            result.failed_reason    = 'no_camera_frame'
            goal_handle.abort()
            return result

        detections = self._detect_objects(frame)
        if not detections:
            self.get_logger().warn('No objects detected')
            if return_home:
                self._home_servos()
            result.success          = False
            result.objects_inspected = 0
            result.failed_reason    = 'no_detection'
            goal_handle.abort()
            return result

        if max_objects > 0:
            detections = detections[:max_objects]

        self.get_logger().info(f'Detected {len(detections)} object(s)')

        # Step 2: inspect each object
        n_inspected = 0
        for obj_idx, (cx_obj, cy_obj, conf, cls_id) in enumerate(detections, start=1):

            if goal_handle.is_cancel_requested:
                break

            self.get_logger().info(f'Object {obj_idx}: cx={cx_obj:.0f} cy={cy_obj:.0f} conf={conf:.2f}')

            # Coarse positioning
            feedback.current_step   = 'coarse'
            feedback.current_object = obj_idx
            goal_handle.publish_feedback(feedback)

            tilt_coarse = coarse_tilt(cy_obj)
            pan_coarse  = coarse_pan(cx_obj)
            self._send_servo(tilt_coarse, pan_coarse)
            self.get_logger().info(f'  Coarse: tilt={tilt_coarse:.1f} pan={pan_coarse:.1f}')
            time.sleep(0.5)  # wait for servo to reach position

            # IBVS fine centering
            feedback.current_step = 'ibvs'
            goal_handle.publish_feedback(feedback)
            centred = self._ibvs_center(goal_handle, obj_idx, feedback)

            if not centred:
                result.success          = False
                result.objects_inspected = n_inspected
                result.failed_reason    = 'ibvs_timeout'
                if return_home:
                    self._home_servos()
                goal_handle.abort()
                return result

            # Capture images
            feedback.current_step = 'capturing'
            goal_handle.publish_feedback(feedback)
            time.sleep(self.RECENTER_WAIT)
            images = self._capture_images(self.IMAGES_PER_OBJ)

            # Publish via MQTT
            feedback.current_step = 'publishing'
            goal_handle.publish_feedback(feedback)
            ok = self._publish_mqtt(images, obj_idx)
            if not ok:
                result.success          = False
                result.objects_inspected = n_inspected
                result.failed_reason    = 'mqtt_error'
                if return_home:
                    self._home_servos()
                goal_handle.abort()
                return result

            n_inspected += 1
            self.get_logger().info(f'Object {obj_idx} done ({n_inspected} total)')

        # All done
        if return_home:
            self._home_servos()

        result.success          = True
        result.objects_inspected = n_inspected
        result.failed_reason    = ''
        goal_handle.succeed()
        self.get_logger().info(f'Inspection complete: {n_inspected} object(s) inspected')
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
