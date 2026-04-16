#!/usr/bin/env python3
"""
go2_webrtc_bridge — cmd_vel_bridge_node
========================================
ROS 2 node that subscribes to /cmd_vel (geometry_msgs/Twist) and forwards
velocity commands to the Unitree GO2 robot via WebRTC, using the same
connection logic as joystick.py.

Parameters
----------
robot_ip        : str   — Robot IP address (default: 192.168.123.161)
cmd_timeout     : float — Seconds before a single command times out (default: 5.0)
reconnect_delay : float — Seconds between reconnection attempts (default: 3.0)
max_speed       : float — Maximum speed scaling factor (default: 1.0)
"""

import asyncio
import json
import logging
import threading
import time

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ---------------------------------------------------------------------------
# Attempt to import the go2_webrtc_driver.  If it is not installed, the node
# will still start but will log a fatal error.
# ---------------------------------------------------------------------------
try:
    from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
    from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD
    GO2_DRIVER_AVAILABLE = True
except ImportError as _e:
    GO2_DRIVER_AVAILABLE = False
    _IMPORT_ERROR = str(_e)


# ---------------------------------------------------------------------------
# ConnectionManager — adapted from joystick.py
# ---------------------------------------------------------------------------
class ConnectionManager:
    """Manages the WebRTC connection with automatic reconnection."""

    def __init__(self, ip: str, cmd_timeout: float, reconnect_delay: float, logger):
        self.ip = ip
        self.cmd_timeout = cmd_timeout
        self.reconnect_delay = reconnect_delay
        self.log = logger
        self.conn = None
        self.is_connected = False
        self.consecutive_failures = 0
        self._lock = asyncio.Lock()
        self.last_success = time.time()

    # ------------------------------------------------------------------
    async def connect(self) -> bool:
        async with self._lock:
            try:
                self.log.info(f"Connecting to GO2 at {self.ip} …")
                if self.conn:
                    try:
                        await self.conn.disconnect()
                    except Exception:
                        pass
                    self.conn = None

                self.conn = Go2WebRTCConnection(
                    WebRTCConnectionMethod.LocalSTA, ip=self.ip
                )
                await asyncio.wait_for(self.conn.connect(), timeout=30)
                self.is_connected = True
                self.consecutive_failures = 0
                self.last_success = time.time()
                self.log.info("Connected to GO2 robot ✓")
                return True
            except asyncio.TimeoutError:
                self.log.error("Connection timed out")
                self.is_connected = False
                return False
            except SystemExit:
                self.log.error("Driver attempted sys.exit() — caught")
                self.is_connected = False
                return False
            except Exception as exc:
                self.log.error(f"Connection failed: {exc}")
                self.is_connected = False
                return False

    # ------------------------------------------------------------------
    async def ensure_connected(self) -> bool:
        if self.is_connected and self.conn and self.conn.isConnected:
            return True
        self.is_connected = False
        self.log.warning("Connection lost — reconnecting …")
        while True:
            if await self.connect():
                return True
            self.log.info(f"Retrying in {self.reconnect_delay}s …")
            await asyncio.sleep(self.reconnect_delay)

    # ------------------------------------------------------------------
    def _build_payload(self, data: dict) -> dict:
        generated_id = int(time.time() * 1000) % 2_147_483_648
        payload = {
            "header": {
                "identity": {
                    "id": data.get("id", generated_id),
                    "api_id": data.get("api_id", 0),
                }
            },
            "parameter": "",
        }
        if "parameter" in data:
            param = data["parameter"]
            payload["parameter"] = param if isinstance(param, str) else json.dumps(param)
        return payload

    # ------------------------------------------------------------------
    async def send_command(self, topic: str, data: dict) -> bool:
        if not await self.ensure_connected():
            return False
        try:
            self.conn.datachannel.pub_sub.publish_without_callback(
                topic, self._build_payload(data)
            )
            self.last_success = time.time()
            self.consecutive_failures = 0
            return True
        except Exception as exc:
            self.consecutive_failures += 1
            self.log.warning(f"Command failed: {exc}")
            if self.consecutive_failures >= 3:
                self.is_connected = False
            return False

    # ------------------------------------------------------------------
    async def disconnect(self):
        if self.conn:
            try:
                await self.conn.disconnect()
            except Exception:
                pass
        self.conn = None
        self.is_connected = False
        self.log.info("Disconnected from GO2")


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------
class CmdVelBridgeNode(Node):
    """
    Subscribes to /cmd_vel and forwards commands to the GO2 via WebRTC.

    Mapping
    -------
    linear.x  → forward/backward speed  (x in GO2 Move API)
    linear.y  → lateral speed           (y in GO2 Move API)
    angular.z → yaw rate                (z in GO2 Move API)
    """

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # ---- Parameters --------------------------------------------------
        self.declare_parameter('robot_ip', '192.168.123.161')
        self.declare_parameter('cmd_timeout', 5.0)
        self.declare_parameter('reconnect_delay', 3.0)
        self.declare_parameter('max_speed', 1.0)

        self._robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self._cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        self._reconnect_delay = self.get_parameter('reconnect_delay').get_parameter_value().double_value
        self._max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        self.get_logger().info(f"robot_ip      = {self._robot_ip}")
        self.get_logger().info(f"cmd_timeout   = {self._cmd_timeout}s")
        self.get_logger().info(f"reconnect_delay = {self._reconnect_delay}s")
        self.get_logger().info(f"max_speed     = {self._max_speed}")

        if not GO2_DRIVER_AVAILABLE:
            self.get_logger().fatal(
                f"go2_webrtc_driver is NOT installed: {_IMPORT_ERROR}\n"
                "Install it with:  pip install -e "
                "/home/yasiru/Documents/github/GO2_WEBRTC_CONTROL/go2_webrtc_connect/"
            )

        # ---- State -------------------------------------------------------
        self._x_speed = 0.0
        self._y_speed = 0.0
        self._z_speed = 0.0
        self._cmd_lock = threading.Lock()
        self._running = True

        self._current_gait = -1
        self._target_gait = -1
        self._gait_lock = threading.Lock()

        # ---- Publishers / Subscribers / Services -------------------------
        self._connected_pub = self.create_publisher(Bool, 'go2_bridge/connected', 10)
        self._cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10
        )
        self._gait_srv = self.create_service(
            AddTwoInts, 'set_gait', self._set_gait_callback
        )

        # ---- Asyncio event loop in background thread ---------------------
        self._loop = asyncio.new_event_loop()
        self._bg_thread = threading.Thread(
            target=self._run_event_loop, daemon=True, name='go2_webrtc_loop'
        )
        self._bg_thread.start()

        # ---- Status timer (publishes /go2_bridge/connected at 1 Hz) ------
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info("cmd_vel_bridge node started — waiting for /cmd_vel messages")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _set_gait_callback(self, request, response):
        """Sets the new target gait."""
        with self._gait_lock:
            self._target_gait = request.a
            
        self.get_logger().info(f"Target gait set to {request.a}. Waiting for robot to transition.")
        response.sum = request.a  # Echo target gait to acknowledge
        return response

    def _cmd_vel_callback(self, msg: Twist):
        """Store latest velocity command (thread-safe)."""
        with self._cmd_lock:
            self._x_speed = max(-self._max_speed,
                                min(self._max_speed, msg.linear.x))
            self._y_speed = max(-self._max_speed,
                                min(self._max_speed, msg.linear.y))
            self._z_speed = max(-self._max_speed,
                                min(self._max_speed, msg.angular.z))
        self.get_logger().debug(
            f"cmd_vel → x={self._x_speed:.3f}  y={self._y_speed:.3f}  z={self._z_speed:.3f}"
        )

    def _publish_status(self):
        msg = Bool()
        if hasattr(self, '_conn_mgr'):
            msg.data = self._conn_mgr.is_connected
        else:
            msg.data = False
        self._connected_pub.publish(msg)

    # ------------------------------------------------------------------
    # Background asyncio loop
    # ------------------------------------------------------------------
    def _run_event_loop(self):
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._command_loop())
        except Exception as exc:
            self.get_logger().error(f"Event loop crashed: {exc}")

    async def _command_loop(self):
        """
        Continuously reads the latest cmd_vel and sends commands to the robot.
        Runs at ~50 Hz.
        """
        if not GO2_DRIVER_AVAILABLE:
            self.get_logger().error("go2_webrtc_driver unavailable — command loop idle")
            while self._running:
                await asyncio.sleep(1.0)
            return

        self._conn_mgr = ConnectionManager(
            ip=self._robot_ip,
            cmd_timeout=self._cmd_timeout,
            reconnect_delay=self._reconnect_delay,
            logger=self.get_logger(),
        )

        # Initial connection (retries indefinitely)
        await self._conn_mgr.ensure_connected()
        self.get_logger().info("Command loop running at 50 Hz")

        # Local callback to track the live gait state from the robot over WebRTC
        def sportmodestatus_callback(message):
            try:
                gait = message['data']['gait_type']
                with self._gait_lock:
                    self._current_gait = gait
            except (KeyError, TypeError):
                pass
        
        # Subscribe to sportmodestat callback
        if self._conn_mgr.conn and self._conn_mgr.conn.datachannel:
            self._conn_mgr.conn.datachannel.pub_sub.subscribe(
                RTC_TOPIC['LF_SPORT_MOD_STATE'], sportmodestatus_callback
            )

        while self._running:
            try:
                with self._cmd_lock:
                    x = self._x_speed
                    y = self._y_speed
                    z = self._z_speed
                
                with self._gait_lock:
                    current_gait = self._current_gait
                    target_gait = self._target_gait

                # Check if we need to switch the gait
                if target_gait != -1 and current_gait != target_gait:
                    # Keep continuously sending SwitchGait until it matches
                    await self._conn_mgr.send_command(
                        RTC_TOPIC["SPORT_MOD"],
                        {
                            "api_id": SPORT_CMD["SwitchGait"],
                            "parameter": {"data": target_gait}
                        }
                    )
                    # Use a slightly longer delay after gait change commands to allow the robot physical time to shift states
                    await asyncio.sleep(0.5)
                    continue

                # Threshold: treat very small values as zero to send StopMove
                dead = 0.01
                if abs(x) < dead and abs(y) < dead and abs(z) < dead:
                    await self._conn_mgr.send_command(
                        RTC_TOPIC["SPORT_MOD"],
                        {"api_id": SPORT_CMD["StopMove"]},
                    )
                else:
                    await self._conn_mgr.send_command(
                        RTC_TOPIC["SPORT_MOD"],
                        {
                            "api_id": SPORT_CMD["Move"],
                            "parameter": {"x": x, "y": y, "z": z},
                        },
                    )

                await asyncio.sleep(0.02)   # 50 Hz

            except Exception as exc:
                self.get_logger().error(f"Command loop error: {exc}")
                self._conn_mgr.is_connected = False
                await asyncio.sleep(1.0)
                
                # Resubscribe on reconnect
                if await self._conn_mgr.ensure_connected() and self._conn_mgr.conn and self._conn_mgr.conn.datachannel:
                    self._conn_mgr.conn.datachannel.pub_sub.subscribe(
                        RTC_TOPIC['LF_SPORT_MOD_STATE'], sportmodestatus_callback
                    )

        await self._conn_mgr.disconnect()

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("Shutting down cmd_vel_bridge …")
        self._running = False
        # Give the background loop a moment to exit cleanly
        self._bg_thread.join(timeout=3.0)
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
