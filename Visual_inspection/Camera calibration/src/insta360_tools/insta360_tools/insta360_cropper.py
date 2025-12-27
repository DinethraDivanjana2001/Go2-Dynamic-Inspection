import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Insta360Cropper(Node):
    def __init__(self):
        super().__init__('insta360_cropper')

        # Declare parameter (front or back)
        self.declare_parameter('camera_half', 'front')
        self.camera_half = self.get_parameter('camera_half').get_parameter_value().string_value

        self.bridge = CvBridge()

        # Subscribe to raw feed (from gscam or cam2image)
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.listener_callback,
            10)

        # Publish cropped output
        self.publisher = self.create_publisher(Image, '/image_cropped', 10)

        self.get_logger().info(f'Insta360 Cropper started with camera_half={self.camera_half}')

    def listener_callback(self, msg):
        # Convert ROS2 image -> OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        h, w, _ = cv_image.shape
        if self.camera_half == 'front':
            cropped = cv_image[:h//2, :]  # Top half
        else:
            cropped = cv_image[h//2:, :]  # Bottom half

        # Publish cropped image
        ros_image = self.bridge.cv2_to_imgmsg(cropped, "bgr8")
        self.publisher.publish(ros_image)

        # Optional: show preview
        cv2.imshow("Cropped View", cropped)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Insta360Cropper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

