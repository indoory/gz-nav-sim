#!/usr/bin/env python3
"""ROS 2 ↔ Isaac Sim (xlerobot_v1 ZMQ) bridge — multi-robot fleet aware.

Replaces the Gazebo backend. The downstream stack (slam_toolbox, Nav2,
foxglove, ros_adapter) sees the same ROS topic interface — only the
producer changes. We bind ONE ROS stack to ONE Isaac robot in the fleet
(parameter `robot_id`, default 0).

Wire spec (indoory_isaac_sim, multi-robot):
  * SUB :5555  sim → robot   sensor PUB    multipart [topic, msgpack]
                             topics suffixed `.<robot_id>`
                             e.g. proprio.0, rgb.front.0, scan.0
  * PUSH :5556 robot → sim   action frame  msgpack {robot_id,
                                                    arm_joint_pos_target(14),
                                                    base_cmd_vel(3)}
  * REQ :5557  RPC           reset / set_pose(robot_id) / enable_stream / ...

Topics consumed (for our robot_id only):
  proprio.<id>      → /odom + odom→base_link TF + /clock
  rgb.front.<id>    → /camera/image_raw + /camera/image_raw/compressed
                       + /camera/camera_info
  depth.front.<id>  → /d456/depth/image_raw + /d456/depth/camera_info
  scan.<id>         → /scan
  (rgb.wrist / depth.wrist / scan.mid / tf.links — ignored, not used by SLAM/Nav)

Subscribed (from ROS):
  /cmd_vel  → base_cmd_vel = [vx, vy, wz], arm = zeros (home pose),
              robot_id = our `robot_id`. `frame` field omitted
              → defaults to "body" (sim yaw-rotates per current pose,
                 which is what Nav2 controller_server already expects).
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Optional

import msgpack
import numpy as np
import rclpy
import zmq
from cv_bridge import CvBridge
import cv2

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from tf2_ros import TransformBroadcaster

try:
    import zstandard as zstd
    _ZSTD_DECOMPRESS = zstd.ZstdDecompressor().decompress
except ImportError:
    _ZSTD_DECOMPRESS = None


SCHEMA = 'xlerobot_v1'
ARM_DOF = 14
BASE_DOF = 3


def _ns_to_time(ns: int) -> TimeMsg:
    sec, nanosec = divmod(int(ns), 1_000_000_000)
    msg = TimeMsg()
    msg.sec = int(sec)
    msg.nanosec = int(nanosec)
    return msg


class IsaacBridge(Node):
    def __init__(self) -> None:
        super().__init__('isaac_bridge')

        # ── connection ──────────────────────────────────────────────────
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('pub_port', 5555)
        self.declare_parameter('push_port', 5556)
        self.declare_parameter('rep_port', 5557)
        self.declare_parameter('cmd_rate_hz', 20.0)
        # Which robot in the Isaac fleet we drive. Default 1 (we usually run
        # robot 1 — robot 0 is reserved for other peers in the lab fleet).
        # sim_server caps at MAX_NUM_ROBOTS=16 and rejects out-of-fleet ids
        # at runtime. Use RPC `fleet_info` to confirm sim's --num-robots.
        self.declare_parameter('robot_id', 1)

        # ── frames ──────────────────────────────────────────────────────
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_optical_frame', 'camera_optical_frame')
        self.declare_parameter('lidar_frame', 'base_link')
        # Which link in tf.links.<id> the camera is mounted on. We re-publish
        # this link's pose as the dynamic TF base_link→camera_frame so
        # nvblox/SLAM see the real Isaac camera placement (instead of a
        # hardcoded mast offset). camera_frame→camera_optical_frame stays
        # static in launch (optical convention rotation).
        self.declare_parameter('camera_link_name', 'head_tilt')

        # ── camera intrinsics (RGB) — Isaac default profile 1280×720 ───
        # Defaults approximate D456 1280×720 RealSense intrinsics
        # (HFOV ~87° → fx ~644). Override via params if Isaac config differs.
        self.declare_parameter('rgb_width', 1280)
        self.declare_parameter('rgb_height', 720)
        self.declare_parameter('rgb_fx', 644.5)
        self.declare_parameter('rgb_fy', 644.5)
        self.declare_parameter('rgb_cx', 640.0)
        self.declare_parameter('rgb_cy', 360.0)

        # depth intrinsics — Isaac default 1280×720 (same as RGB front)
        self.declare_parameter('depth_width', 1280)
        self.declare_parameter('depth_height', 720)
        self.declare_parameter('depth_fx', 644.5)
        self.declare_parameter('depth_fy', 644.5)
        self.declare_parameter('depth_cx', 640.0)
        self.declare_parameter('depth_cy', 360.0)
        # mm → m. Isaac sends uint16 mm by default (depth_scale_m=0.001).
        self.declare_parameter('depth_scale_m', 0.001)

        # ── lidar geometry — match Gazebo D456 model (-π .. π, 12 m) ───
        # scan_range_min defaults to 0.20 m: drop self-returns where the lidar
        # picks up the robot's own chassis. Anything closer is rewritten to
        # inf so slam_toolbox / nav2 obstacle_layer treat it as no-return.
        self.declare_parameter('scan_angle_min', -3.14159)
        self.declare_parameter('scan_angle_max', 3.14159)
        self.declare_parameter('scan_range_min', 0.20)
        self.declare_parameter('scan_range_max', 12.0)
        # Isaac currently reports useful base_pose deltas while base_twist may
        # stay near zero. Use pose deltas for odom so Nav2 sees actual motion.
        self.declare_parameter('odom_from_pose_delta', True)

        g = lambda n: self.get_parameter(n).value
        self._host = str(g('host'))
        self._pub_port = int(g('pub_port'))
        self._push_port = int(g('push_port'))
        self._rep_port = int(g('rep_port'))
        self._cmd_period = 1.0 / max(1.0, float(g('cmd_rate_hz')))
        self._robot_id = int(g('robot_id'))
        # Per-robot topic names — match against incoming SUB frames.
        self._t_proprio = f'proprio.{self._robot_id}'
        self._t_rgb = f'rgb.front.{self._robot_id}'
        self._t_depth = f'depth.front.{self._robot_id}'
        self._t_scan = f'scan.{self._robot_id}'
        self._t_tflinks = f'tf.links.{self._robot_id}'

        self._odom_frame = str(g('odom_frame'))
        self._base_frame = str(g('base_frame'))
        self._cam_frame = str(g('camera_optical_frame'))
        self._lidar_frame = str(g('lidar_frame'))
        self._camera_link_name = str(g('camera_link_name'))

        self._scan_angle_min = float(g('scan_angle_min'))
        self._scan_angle_max = float(g('scan_angle_max'))
        self._scan_range_min = float(g('scan_range_min'))
        self._scan_range_max = float(g('scan_range_max'))
        self._depth_scale = float(g('depth_scale_m'))
        self._odom_from_pose_delta = bool(g('odom_from_pose_delta'))

        # Pre-build CameraInfo templates — only stamp/frame change per frame.
        self._rgb_info_tmpl = self._build_camera_info(
            int(g('rgb_width')), int(g('rgb_height')),
            float(g('rgb_fx')), float(g('rgb_fy')),
            float(g('rgb_cx')), float(g('rgb_cy')),
        )
        self._depth_info_tmpl = self._build_camera_info(
            int(g('depth_width')), int(g('depth_height')),
            float(g('depth_fx')), float(g('depth_fy')),
            float(g('depth_cx')), float(g('depth_cy')),
        )

        # ── ROS pubs / subs ─────────────────────────────────────────────
        # Match Gazebo plugin defaults: /odom + /scan are RELIABLE (slam_toolbox,
        # nav2 costmaps, telemetry adapters expect reliable). Camera/depth stay
        # BEST_EFFORT for throughput.
        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        rel_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self._pub_clock = self.create_publisher(Clock, '/clock', 10)
        self._pub_odom = self.create_publisher(Odometry, '/odom', rel_qos)
        self._pub_scan = self.create_publisher(LaserScan, '/scan', rel_qos)
        self._pub_rgb = self.create_publisher(Image, '/camera/image_raw', sensor_qos)
        self._pub_rgb_compressed = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', sensor_qos)
        self._pub_rgb_info = self.create_publisher(CameraInfo, '/camera/camera_info', rel_qos)
        self._pub_depth = self.create_publisher(Image, '/d456/depth/image_raw', sensor_qos)
        self._pub_depth_info = self.create_publisher(CameraInfo, '/d456/depth/camera_info', rel_qos)

        self._tf_bcast = TransformBroadcaster(self)
        self._cv_bridge = CvBridge()

        self._cmd_lock = threading.Lock()
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        # SO-ARM101 leader 14 joint target — adapter 가 50Hz publish.
        # Float64MultiArray.data: 좌 6 + 우 6 + 2 (head/neck, 보류) = 14.
        # 메시지 미수신 시 home pose [0]*14 유지.
        from std_msgs.msg import Float64MultiArray
        self._arm_target = [0.0] * ARM_DOF
        self.create_subscription(Float64MultiArray, '/leader_arm_joint_target',
                                 self._on_arm_target, 10)

        # ── ZMQ ──────────────────────────────────────────────────────────
        self._zmq_ctx = zmq.Context.instance()
        self._sub = self._zmq_ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.RCVHWM, 8)
        self._sub.setsockopt(zmq.LINGER, 0)
        self._sub.connect(f'tcp://{self._host}:{self._pub_port}')
        # Subscribe selectively to *our* robot's 5 topics. ZMQ does prefix
        # matching, but the strings are exact-name and don't share prefixes
        # with other-robot or wrist/mid topics, so the kernel filters out
        # the 1280×720 jpeg traffic for the other N-1 robots automatically.
        for t in (self._t_proprio, self._t_rgb, self._t_depth, self._t_scan,
                  self._t_tflinks):
            self._sub.setsockopt(zmq.SUBSCRIBE, t.encode('ascii'))

        self._push = self._zmq_ctx.socket(zmq.PUSH)
        self._push.setsockopt(zmq.SNDHWM, 4)
        self._push.setsockopt(zmq.LINGER, 0)
        self._push.connect(f'tcp://{self._host}:{self._push_port}')

        self.get_logger().info(
            f'isaac_bridge[robot_id={self._robot_id}] connected: '
            f'SUB tcp://{self._host}:{self._pub_port} '
            f'PUSH tcp://{self._host}:{self._push_port}  '
            f'topics=[{self._t_proprio}, {self._t_rgb}, {self._t_depth}, {self._t_scan}]')
        if _ZSTD_DECOMPRESS is None:
            self.get_logger().warn(
                "zstandard not installed — depth.front frames will be skipped. "
                "Install via `pip3 install zstandard`.")

        # SUB thread runs blocking poll in background; ROS pubs are thread-safe.
        self._stop = threading.Event()
        self._seen_topics: set[str] = set()

        # ── odom integrator ─────────────────────────────────────────────
        # Prefer pose deltas in Isaac mode because some sim_server builds keep
        # base_twist at zero even while the base is moving. Large jumps are
        # treated as resets/teleports and are not integrated.
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._prev_stamp_ns: Optional[int] = None
        self._prev_world_pose: Optional[tuple[float, float, float]] = None
        self._sub_thread = threading.Thread(target=self._sub_loop, daemon=True)
        self._sub_thread.start()

        # PUSH on a timer so /cmd_vel is never stale (matches keyboard_client).
        self._zero_arm = [0.0] * ARM_DOF
        self.create_timer(self._cmd_period, self._tick_push)

    # ── ZMQ → ROS ────────────────────────────────────────────────────────
    def _sub_loop(self) -> None:
        poller = zmq.Poller()
        poller.register(self._sub, zmq.POLLIN)
        while not self._stop.is_set():
            try:
                socks = dict(poller.poll(timeout=200))
            except zmq.ContextTerminated:
                return
            if self._sub not in socks:
                continue
            try:
                topic_b, payload = self._sub.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                continue
            except zmq.ZMQError as exc:
                self.get_logger().warn(f'zmq recv error: {exc}')
                continue

            try:
                msg = msgpack.unpackb(payload, raw=False)
            except Exception as exc:  # malformed
                self.get_logger().warn(f'msgpack decode failed: {exc}')
                continue
            if not isinstance(msg, dict):
                continue
            # Schema only enforced when the payload explicitly carries one.
            # In practice sim_server omits it on sensor topics (rgb/depth/scan)
            # and only adds it to proprio + RPC. Dropping unconditionally here
            # silently swallowed every camera/lidar frame.
            schema = msg.get('schema')
            if schema is not None and schema != SCHEMA:
                continue

            topic = topic_b.decode('ascii', errors='replace')
            if topic not in self._seen_topics:
                self._seen_topics.add(topic)
                keys = sorted(k for k in msg.keys() if k != 'data')
                self.get_logger().info(
                    f'first frame on {topic!r}: keys={keys}')
            try:
                self._dispatch(topic, msg)
            except Exception as exc:
                self.get_logger().error(
                    f'dispatch {topic}: {type(exc).__name__}: {exc}')

    def _dispatch(self, topic: str, msg: dict) -> None:
        if topic == self._t_proprio:
            self._handle_proprio(msg)
        elif topic == self._t_rgb:
            self._handle_rgb(msg)
        elif topic == self._t_depth:
            self._handle_depth(msg)
        elif topic == self._t_scan:
            # main lidar (z=0.10) — feeds slam_toolbox + nav2 costmaps.
            self._handle_scan(msg)
        elif topic == self._t_tflinks:
            self._handle_tf_links(msg)
        # scan.mid.<id>, rgb.wrist.<id>, depth.wrist.<id> intentionally
        # dropped — not used by SLAM/Nav.

    def _handle_proprio(self, msg: dict) -> None:
        stamp_ns = int(msg.get('stamp_ns', 0))
        stamp = _ns_to_time(stamp_ns)

        # /clock — wire the rest of the stack to sim time.
        clock = Clock()
        clock.clock = stamp
        self._pub_clock.publish(clock)

        pose = msg.get('base_pose') or [0.0] * 7
        twist = msg.get('base_twist') or [0.0] * 6
        if len(pose) < 7 or len(twist) < 6:
            return
        px_w = float(pose[0])
        py_w = float(pose[1])
        qx, qy, qz, qw = (float(v) for v in pose[3:7])
        vx_w = float(twist[0])
        vy_w = float(twist[1])
        wz = float(twist[5])

        # World yaw from the world-frame quaternion (z-axis rotation only,
        # ground robot — pitch/roll ignored).
        yaw_w = math.atan2(2.0 * (qw * qz + qx * qy),
                           1.0 - 2.0 * (qy * qy + qz * qz))

        # Δt from previous proprio stamp; first frame has no integration step.
        if self._prev_stamp_ns is None:
            dt = 0.0
        else:
            dt = max(0.0, (stamp_ns - self._prev_stamp_ns) * 1e-9)
        self._prev_stamp_ns = stamp_ns

        if self._odom_from_pose_delta and self._prev_world_pose is not None:
            prev_x_w, prev_y_w, prev_yaw_w = self._prev_world_pose
            dx_w = px_w - prev_x_w
            dy_w = py_w - prev_y_w
            dyaw_w = math.atan2(
                math.sin(yaw_w - prev_yaw_w),
                math.cos(yaw_w - prev_yaw_w))
            # Ignore teleports/resets and stale frames; otherwise derive odom
            # velocity from the measured pose delta.
            if 1e-4 <= dt <= 1.0 and math.hypot(dx_w, dy_w) < 1.0 and abs(dyaw_w) < 1.5:
                vx_w = dx_w / dt
                vy_w = dy_w / dt
                wz = dyaw_w / dt
        self._prev_world_pose = (px_w, py_w, yaw_w)

        # Rotate world-frame velocity by -yaw_w → body frame.
        cs_w, sn_w = math.cos(yaw_w), math.sin(yaw_w)
        vx_b = vx_w * cs_w + vy_w * sn_w
        vy_b = -vx_w * sn_w + vy_w * cs_w

        # Integrate body-frame velocity through our drifty odom-yaw.
        cs_o, sn_o = math.cos(self._odom_yaw), math.sin(self._odom_yaw)
        self._odom_x += (vx_b * cs_o - vy_b * sn_o) * dt
        self._odom_y += (vx_b * sn_o + vy_b * cs_o) * dt
        self._odom_yaw += wz * dt

        half = self._odom_yaw * 0.5
        odom_qx, odom_qy = 0.0, 0.0
        odom_qz, odom_qw = math.sin(half), math.cos(half)

        # /odom — pose in odom frame, twist in body frame (ROS convention:
        # twist refers to child_frame_id which is base_link).
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(
            x=odom_qx, y=odom_qy, z=odom_qz, w=odom_qw)
        odom.twist.twist.linear = Vector3(x=vx_b, y=vy_b, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=wz)
        self._pub_odom.publish(odom)

        # odom → base_link TF
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = self._odom_x
        tf.transform.translation.y = self._odom_y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = Quaternion(
            x=odom_qx, y=odom_qy, z=odom_qz, w=odom_qw)
        self._tf_bcast.sendTransform(tf)

    def _handle_rgb(self, msg: dict) -> None:
        data = msg.get('data')
        if not data:
            return
        encoding = msg.get('encoding', 'jpeg')
        if encoding != 'jpeg':
            self.get_logger().warn(f'rgb.front: unsupported encoding {encoding!r}')
            return
        stamp = _ns_to_time(int(msg.get('stamp_ns', 0)))

        # JPEG → numpy → Image (rgb8). Also republish the raw JPEG as
        # CompressedImage for foxglove/bandwidth-sensitive consumers.
        buf = np.frombuffer(bytes(data), dtype=np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warn('rgb.front: jpeg decode failed')
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        img = self._cv_bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        img.header.stamp = stamp
        img.header.frame_id = self._cam_frame
        self._pub_rgb.publish(img)

        comp = CompressedImage()
        comp.header.stamp = stamp
        comp.header.frame_id = self._cam_frame
        comp.format = 'jpeg'
        comp.data = bytes(data)
        self._pub_rgb_compressed.publish(comp)

        self._pub_rgb_info.publish(
            self._stamp_camera_info(self._rgb_info_tmpl, stamp, self._cam_frame))

    def _handle_depth(self, msg: dict) -> None:
        if _ZSTD_DECOMPRESS is None:
            return
        raw = msg.get('data')
        if not raw:
            return
        encoding = msg.get('encoding', 'u16_zstd')
        if encoding != 'u16_zstd':
            self.get_logger().warn(f'depth.front: unsupported encoding {encoding!r}')
            return
        width = int(msg.get('width', 0))
        height = int(msg.get('height', 0))
        if width <= 0 or height <= 0:
            return
        stamp = _ns_to_time(int(msg.get('stamp_ns', 0)))
        scale_m = float(msg.get('depth_scale_m', self._depth_scale))

        try:
            decompressed = _ZSTD_DECOMPRESS(bytes(raw))
        except Exception as exc:
            self.get_logger().warn(f'depth.front zstd decode failed: {exc}')
            return

        arr = np.frombuffer(decompressed, dtype=np.uint16)
        if arr.size != width * height:
            self.get_logger().warn(
                f'depth.front size mismatch: got {arr.size}, '
                f'expected {width * height} ({width}x{height})')
            return
        depth = arr.reshape(height, width)

        # Publish as 16UC1 millimetres — what RTAB-Map / nvblox expect by
        # default. If sim sends a non-mm scale, callers can convert via the
        # `depth_scale_m` param downstream (most consumers assume mm).
        if abs(scale_m - 0.001) > 1e-9:
            # Re-express as mm uint16 (clip to range to avoid overflow).
            depth = np.clip(
                depth.astype(np.float32) * scale_m * 1000.0,
                0.0, 65535.0).astype(np.uint16)

        img = self._cv_bridge.cv2_to_imgmsg(depth, encoding='16UC1')
        img.header.stamp = stamp
        img.header.frame_id = self._cam_frame
        self._pub_depth.publish(img)

        info = self._stamp_camera_info(self._depth_info_tmpl, stamp, self._cam_frame)
        self._pub_depth_info.publish(info)

    def _handle_scan(self, msg: dict) -> None:
        ranges_raw = msg.get('ranges')
        if ranges_raw is None:
            return
        # sim_server packs ranges as msgpack bin (raw float32 bytes), not as
        # a Python list. Iterating bytes directly yields 0-255 ints — which
        # gets silently published as garbage. Detect both shapes.
        if isinstance(ranges_raw, (bytes, bytearray, memoryview)):
            ranges_arr = np.frombuffer(bytes(ranges_raw), dtype=np.float32)
        else:
            ranges_arr = np.asarray(list(ranges_raw), dtype=np.float32)
        n = int(ranges_arr.size)
        if n == 0:
            return

        stamp = _ns_to_time(int(msg.get('stamp_ns', 0)))
        scan = LaserScan()
        scan.header.stamp = stamp
        # Force our own lidar_frame ("base_link" by default). sim_server tags
        # scans with internal frames like "mid_scan" / "lidar_link" that have
        # no static TF in our launch — slam_toolbox's tf2 message filter then
        # drops every scan with "queue is full" and SLAM never advances.
        scan.header.frame_id = self._lidar_frame
        # Allow per-message overrides; otherwise fall back to the params.
        amin = float(msg.get('angle_min', self._scan_angle_min))
        amax = float(msg.get('angle_max', self._scan_angle_max))
        scan.angle_min = amin
        scan.angle_max = amax
        # CRITICAL: take sim's own angle_increment if provided. If we recompute
        # it from (amax-amin)/n, slam_toolbox's `expected = (amax-amin)/inc`
        # rounds back to n+1 (e.g. 500 vs our 499 rays) and drops every scan.
        scan.angle_increment = float(
            msg.get('angle_increment', (amax - amin) / max(1, n)))
        scan.time_increment = 0.0
        scan.scan_time = float(msg.get('scan_time', 0.1))
        # Force our self-filter range_min instead of sim's (sim sends 0.05).
        # Anything closer than range_min is rewritten to inf so consumers
        # that look at raw ranges (not just header.range_min) drop it too.
        scan.range_min = self._scan_range_min
        scan.range_max = float(msg.get('range_max', self._scan_range_max))
        if self._scan_range_min > 0.0:
            ranges_arr = np.where(
                ranges_arr < self._scan_range_min,
                np.float32(np.inf), ranges_arr)
        scan.ranges = ranges_arr.tolist()
        self._pub_scan.publish(scan)

    def _handle_tf_links(self, msg: dict) -> None:
        """Republish base_link → camera_frame as a dynamic TF.

        Pulls the pose of `camera_link_name` (default `head_tilt`) out of
        the tf.links payload and broadcasts it as TF every 30 Hz. nvblox /
        SLAM see the actual sim camera placement (including head pan/tilt
        motion) instead of a stale static guess.
        """
        targets = msg.get('targets')
        if not targets:
            return
        stamp = _ns_to_time(int(msg.get('stamp_ns', 0)))
        for t in targets:
            if t.get('name') != self._camera_link_name:
                continue
            pose = t.get('pose')
            if not pose or len(pose) < 7:
                return
            x, y, z, qx, qy, qz, qw = (float(v) for v in pose[:7])
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._base_frame
            # Downstream stack already has a static camera_frame →
            # camera_optical_frame TF (optical convention). We feed
            # base_link → camera_frame here so the chain stays the same.
            tf.child_frame_id = 'camera_frame'
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            self._tf_bcast.sendTransform(tf)
            return

    # ── ROS → ZMQ ────────────────────────────────────────────────────────
    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._cmd_lock:
            self._cmd_vx = float(msg.linear.x)
            self._cmd_vy = float(msg.linear.y)
            self._cmd_wz = float(msg.angular.z)

    def _on_arm_target(self, msg) -> None:
        # adapter 의 leader read thread 가 50Hz 로 publish. 길이 != ARM_DOF 이면 pad/truncate.
        data = list(msg.data)[:ARM_DOF]
        if len(data) < ARM_DOF:
            data = data + [0.0] * (ARM_DOF - len(data))
        with self._cmd_lock:
            self._arm_target = data

    def _tick_push(self) -> None:
        with self._cmd_lock:
            base = [self._cmd_vx, self._cmd_vy, self._cmd_wz]
            arm = list(self._arm_target)
        # No `frame` key → defaults to "body" on sim side (yaw-rotated).
        # That matches Nav2 controller_server / teleop conventions where
        # cmd_vel is already in the robot's body frame.
        frame = {
            'schema': SCHEMA,
            'stamp_ns': time.time_ns(),
            'robot_id': self._robot_id,
            'arm_joint_pos_target': arm,
            'base_cmd_vel': base,
        }
        try:
            self._push.send(msgpack.packb(frame, use_bin_type=True), zmq.NOBLOCK)
        except zmq.Again:
            # sim PULL backed up — drop, will resend next tick.
            pass
        except zmq.ZMQError as exc:
            self.get_logger().warn(f'push send error: {exc}')

    # ── helpers ──────────────────────────────────────────────────────────
    @staticmethod
    def _build_camera_info(
            width: int, height: int, fx: float, fy: float,
            cx: float, cy: float) -> CameraInfo:
        info = CameraInfo()
        info.width = int(width)
        info.height = int(height)
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    @staticmethod
    def _stamp_camera_info(tmpl: CameraInfo, stamp: TimeMsg, frame: str) -> CameraInfo:
        info = CameraInfo()
        info.width = tmpl.width
        info.height = tmpl.height
        info.distortion_model = tmpl.distortion_model
        info.d = list(tmpl.d)
        info.k = list(tmpl.k)
        info.r = list(tmpl.r)
        info.p = list(tmpl.p)
        info.header.stamp = stamp
        info.header.frame_id = frame
        return info

    def destroy_node(self) -> None:
        self._stop.set()
        try:
            self._sub.close(linger=0)
        except Exception:
            pass
        try:
            self._push.close(linger=0)
        except Exception:
            pass
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = IsaacBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
