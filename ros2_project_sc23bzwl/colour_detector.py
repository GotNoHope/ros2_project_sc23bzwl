import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # decode the image with its native encoding
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges
        masks = {
            'Red': cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)),
            'Green': cv2.inRange(hsv, (50, 100, 100), (70, 255, 255)),
            'Blue': cv2.inRange(hsv, (100, 100, 100), (130, 255, 255)),
        }

        for color, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:  # Filter small noise
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255,255,255), 2)
                    cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

        cv2.imshow("Color Detection", frame)
        cv2.waitKey(1)
        self.get_logger().info(f"Image encoding: {msg.encoding}")

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
