#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

class ClockRepublisher(Node):
    def __init__(self):
        super().__init__('clock_republisher')
        self.publisher = self.create_publisher(Clock, '/clock', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )
        self.last_stamp = None
        self.min_delta = rclpy.duration.Duration(seconds=0.001)  # 1 ms filter

    def callback(self, msg):
        """Republish the timestamp from /joint_states as /clock, ignoring backward or repeated times."""
        if msg.header.stamp is None:
            return

        new_time = rclpy.time.Time.from_msg(msg.header.stamp)

        # --- Skip first message or backwards time jumps ---
        if self.last_stamp and new_time < self.last_stamp:
            self.get_logger().warn("⚠️ Detected backward clock jump, skipping...")
            return

        # --- Throttle very small deltas (prevents flooding) ---
        if self.last_stamp and (new_time - self.last_stamp) < self.min_delta:
            return

        self.last_stamp = new_time

        clk = Clock()
        clk.clock = msg.header.stamp
        self.publisher.publish(clk)

def main(args=None):
    rclpy.init(args=args)
    node = ClockRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
