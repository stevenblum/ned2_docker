#!/usr/bin/env python3
import base64
import threading
import time
from typing import Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time as Ros2Time

import roslibpy


def to_ros2_time(stamp_dict: Dict[str, int]) -> Ros2Time:
    t = Ros2Time()
    t.sec = int(stamp_dict.get('secs', stamp_dict.get('sec', 0)))
    t.nanosec = int(stamp_dict.get('nsecs', stamp_dict.get('nanosec', 0)))
    return t


def to_ros2_header(h: Dict[str, Any]) -> Header:
    hdr = Header()
    hdr.frame_id = h.get('frame_id', '')
    if 'stamp' in h and isinstance(h['stamp'], dict):
        hdr.stamp = to_ros2_time(h['stamp'])
    return hdr


class RosbridgeCameraBridge(Node):
    """
    Subscribes to ROS1 topics via rosbridge WebSocket and republishes as ROS2 topics.
    """

    def __init__(self,
                 rb_host: str,
                 rb_port: int,
                 topics_image: List[str],
                 topics_cinfo: List[str],
                 topics_image_compressed: List[str],
                 bridge_clock: bool = True,
                 bridge_tf: bool = False):
        super().__init__('rosbridge_camera_bridge')

        # QoS tuned for sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.image_pubs: Dict[str, Any] = {
            t: self.create_publisher(Image, t, sensor_qos) for t in topics_image
        }
        self.cinfo_pubs: Dict[str, Any] = {
            t: self.create_publisher(CameraInfo, t, latched_qos) for t in topics_cinfo
        }
        self.comp_pubs: Dict[str, Any] = {
            t: self.create_publisher(CompressedImage, t, sensor_qos) for t in topics_image_compressed
        }

        self.clock_pub = self.create_publisher(Clock, '/clock', latched_qos) if bridge_clock else None
        self.tf_pub = self.create_publisher(TFMessage, '/tf', sensor_qos) if bridge_tf else None
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', latched_qos) if bridge_tf else None

        # Rosbridge client
        self.rb = roslibpy.Ros(host=rb_host, port=rb_port)
        self._subs: List[roslibpy.Topic] = []
        self._lock = threading.Lock()

        # Create subscriptions
        for t in topics_image:
            self._subs.append(roslibpy.Topic(self.rb, t, 'sensor_msgs/Image'))
        for t in topics_cinfo:
            self._subs.append(roslibpy.Topic(self.rb, t, 'sensor_msgs/CameraInfo'))
        for t in topics_image_compressed:
            self._subs.append(roslibpy.Topic(self.rb, t, 'sensor_msgs/CompressedImage'))

        if bridge_clock:
            self._subs.append(roslibpy.Topic(self.rb, '/clock', 'rosgraph_msgs/Clock'))
        if bridge_tf:
            self._subs.append(roslibpy.Topic(self.rb, '/tf', 'tf2_msgs/TFMessage'))
            self._subs.append(roslibpy.Topic(self.rb, '/tf_static', 'tf2_msgs/TFMessage'))

        # Connect and start
        self._start_rosbridge()

        # Periodic connection watchdog
        self.timer = self.create_timer(2.0, self._watchdog)

    def _start_rosbridge(self):
        self.get_logger().info('Connecting to rosbridge...')
        self.rb.run(3.0)  # non-blocking; start IO thread
        # Wait for connection
        wait_s = 0
        while not self.rb.is_connected and wait_s < 5:
            time.sleep(0.1)
            wait_s += 0.1
        if not self.rb.is_connected:
            self.get_logger().error('Could not connect to rosbridge.')
            return

        # Register callbacks
        for sub in self._subs:
            sub.subscribe(self._make_cb(sub.name, sub.message_type))
            self.get_logger().info(f'Subscribed via rosbridge: {sub.name} [{sub.message_type}]')

    def destroy_node(self):
        with self._lock:
            for sub in self._subs:
                try:
                    sub.unsubscribe()
                except Exception:
                    pass
            try:
                self.rb.terminate()
            except Exception:
                pass
        return super().destroy_node()

    def _watchdog(self):
        # Simple reconnection logic
        if not self.rb.is_connected:
            self.get_logger().warn('rosbridge disconnected, retrying...')
            try:
                self._start_rosbridge()
            except Exception as e:
                self.get_logger().error(f'reconnect failed: {e}')

    def _make_cb(self, topic_name: str, msg_type: str):
        def cb(msg: Dict[str, Any]):
            try:
                if msg_type == 'sensor_msgs/Image':
                    out = Image()
                    out.header = to_ros2_header(msg.get('header', {}))
                    out.height = int(msg['height'])
                    out.width = int(msg['width'])
                    out.encoding = msg['encoding']
                    out.is_bigendian = int(msg.get('is_bigendian', 0))
                    out.step = int(msg['step'])
                    # rosbridge encodes byte arrays as base64 by default
                    data_field = msg.get('data', '')
                    if isinstance(data_field, str):
                        out.data = list(base64.b64decode(data_field))
                    else:
                        # Some rosbridge configs may pass as list[int]
                        out.data = data_field
                    self.image_pubs[topic_name].publish(out)

                elif msg_type == 'sensor_msgs/CompressedImage':
                    out = CompressedImage()
                    out.header = to_ros2_header(msg.get('header', {}))
                    out.format = msg.get('format', '')
                    data_field = msg.get('data', '')
                    if isinstance(data_field, str):
                        out.data = list(base64.b64decode(data_field))
                    else:
                        out.data = data_field
                    self.comp_pubs[topic_name].publish(out)

                elif msg_type == 'sensor_msgs/CameraInfo':
                    out = CameraInfo()
                    out.header = to_ros2_header(msg.get('header', {}))
                    out.height = int(msg.get('height', 0))
                    out.width = int(msg.get('width', 0))
                    out.distortion_model = msg.get('distortion_model', '')
                    out.d = [float(x) for x in msg.get('D', msg.get('d', []))]
                    out.k = [float(x) for x in msg.get('K', [0]*9)]
                    out.r = [float(x) for x in msg.get('R', [0]*9)]
                    out.p = [float(x) for x in msg.get('P', [0]*12)]
                    out.binning_x = int(msg.get('binning_x', 0))
                    out.binning_y = int(msg.get('binning_y', 0))
                    # roi is optional and often empty
                    if 'roi' in msg:
                        roi = msg['roi']
                        out.roi.x_offset = int(roi.get('x_offset', 0))
                        out.roi.y_offset = int(roi.get('y_offset', 0))
                        out.roi.height = int(roi.get('height', 0))
                        out.roi.width = int(roi.get('width', 0))
                        out.roi.do_rectify = bool(roi.get('do_rectify', False))
                    self.cinfo_pubs[topic_name].publish(out)

                elif msg_type == 'rosgraph_msgs/Clock' and self.clock_pub is not None:
                    out = Clock()
                    if 'clock' in msg and isinstance(msg['clock'], dict):
                        out.clock = to_ros2_time(msg['clock'])
                    self.clock_pub.publish(out)

                elif msg_type == 'tf2_msgs/TFMessage':
                    # Optional TF bridging (lightweight, no static filtering)
                    tfs = []
                    for tr in msg.get('transforms', []):
                        # We map directly by field names; rosbridge sends nested dicts
                        from geometry_msgs.msg import TransformStamped
                        ts = TransformStamped()
                        ts.header = to_ros2_header(tr.get('header', {}))
                        ts.child_frame_id = tr.get('child_frame_id', '')
                        t = tr.get('transform', {})
                        ts.transform.translation.x = float(t.get('translation', {}).get('x', 0.0))
                        ts.transform.translation.y = float(t.get('translation', {}).get('y', 0.0))
                        ts.transform.translation.z = float(t.get('translation', {}).get('z', 0.0))
                        ts.transform.rotation.x = float(t.get('rotation', {}).get('x', 0.0))
                        ts.transform.rotation.y = float(t.get('rotation', {}).get('y', 0.0))
                        ts.transform.rotation.z = float(t.get('rotation', {}).get('z', 0.0))
                        ts.transform.rotation.w = float(t.get('rotation', {}).get('w', 1.0))
                        tfs.append(ts)
                    out = TFMessage(transforms=tfs)
                    # Heuristic: static transforms are rare/highly repeated; if you prefer,
                    # publish all to /tf (simpler). Here we just push to /tf.
                    if self.tf_pub is not None:
                        self.tf_pub.publish(out)

            except Exception as e:
                self.get_logger().warn(f'Failed to bridge {topic_name} [{msg_type}]: {e}')
        return cb


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--rb-host', default='localhost', help='rosbridge host/IP (ROS1 sim)')
    parser.add_argument('--rb-port', type=int, default=9090, help='rosbridge websocket port')
    parser.add_argument('--use-sim-time', action='store_true',
                        help='set use_sim_time for this node (subscribe /clock if bridged)')
    args = parser.parse_args()

    # Your topic set from the sim container:
    topics_image = [
        '/cam_left/my_fixed_camera/image_raw',
        '/cam_right/my_fixed_camera/image_raw',
        '/gazebo_camera/image_raw',
    ]
    topics_cinfo = [
        '/cam_left/my_fixed_camera/camera_info',
        '/cam_right/my_fixed_camera/camera_info',
        '/gazebo_camera/camera_info',
    ]
    # Add compressed bridges if/when you want them
    topics_image_compressed = [
        # '/cam_left/my_fixed_camera/image_raw/compressed',
        # '/cam_right/my_fixed_camera/image_raw/compressed',
        # '/gazebo_camera/image_raw/compressed',
    ]

    rclpy.init()
    node = RosbridgeCameraBridge(
        rb_host=args.rb_host,
        rb_port=args.rb_port,
        topics_image=topics_image,
        topics_cinfo=topics_cinfo,
        topics_image_compressed=topics_image_compressed,
        bridge_clock=True,        # you can toggle
        bridge_tf=False           # set True if you also want TF via rosbridge
    )
    if args.use_sim_time:
        node.set_parameters([rclpy.parameter.Parameter('use_sim_time',
                                                       rclpy.parameter.Parameter.Type.BOOL, True)])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
