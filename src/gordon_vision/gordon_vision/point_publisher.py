import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped


class PointPublisher(Node):
    def __init__(self):
        super().__init__("point_publisher")
        self.publisher = self.create_publisher(PointStamped, "point", 10)

        point_stamped = PointStamped()
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.header.frame_id = "camera_link"
        point_stamped.point.x = 0.01
        point_stamped.point.y = 0.01
        point_stamped.point.z = 0.01

        self.publisher.publish(point_stamped)


def main(args=None):
    rclpy.init(args=args)
    point_publisher = PointPublisher()
    rclpy.spin(point_publisher)
    point_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
