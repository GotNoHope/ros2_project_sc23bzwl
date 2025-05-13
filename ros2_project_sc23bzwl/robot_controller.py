import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ManualExplorer(Node):
    def __init__(self):
        super().__init__('manual_explorer')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        self.green_found = False
        self.blue_found = False

        # Step execution state
        self.step = 0
        self.last_step_time = time.time()

        # Movement update loop
        self.timer = self.create_timer(0.1, self.run_step)

    def run_step(self):
        now = time.time()

        def send_twist(linear=0.0, angular=0.0):
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.publisher.publish(twist)

        if self.step == 0:
            self.get_logger().info("Rotating left 3 times...")
            send_twist(angular=0.5)
            if now - self.last_step_time > 6.0:  # Adjust timing if needed
                self.step += 1
                self.last_step_time = now
                send_twist(0.0, 0.0)

        elif self.step == 1:
            self.get_logger().info("Driving forward...")
            send_twist(linear=0.25)
            if now - self.last_step_time > 5.5:
                self.step += 1
                self.last_step_time = now
                send_twist(0.0, 0.0)

        elif self.step == 2:
            self.get_logger().info("Rotating to scan for green/red...")
            send_twist(angular=0.4)
            if self.green_found:
                self.step += 1
                self.last_step_time = now
                send_twist(0.0, 0.0)

        elif self.step == 3:
            self.get_logger().info("Green found. Driving closer...")
            send_twist(linear=0.25)
            if now - self.last_step_time > 3.0:
                self.step += 1
                self.last_step_time = now
                send_twist(0.0, 0.0)

        elif self.step == 4:
            self.get_logger().info("Rotating to find blue...")
            send_twist(angular=0.4)
            if self.blue_found:
                self.step += 1
                self.last_step_time = now
                send_twist(0.0, 0.0)

        elif self.step == 5:
            self.get_logger().info("Driving near blue box...")
            send_twist(linear=0.2)
            if now - self.last_step_time > 2.5:
                send_twist(0.0, 0.0)
                self.get_logger().info("Task Complete. Stopping.")
                self.step += 1  # End
        else:
            send_twist(0.0, 0.0)  # Remain stopped

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CV error: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        green_mask = cv2.inRange(hsv, (50, 100, 100), (70, 255, 255))
        blue_mask = cv2.inRange(hsv, (100, 100, 100), (130, 255, 255))

        if cv2.countNonZero(green_mask) > 800:
            self.green_found = True
        if cv2.countNonZero(blue_mask) > 800:
            self.blue_found = True

        # Optional debug
        cv2.imshow("View", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ManualExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
