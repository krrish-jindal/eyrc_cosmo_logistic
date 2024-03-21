#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_util import LifecycleNodeClient
from geometry_msgs.msg import PolygonStamped, Point32

class ChangeFootprint(Node):

    def __init__(self):
        super().__init__('change_footprint')
        self.lc_client = LifecycleNodeClient()
        self.service_change_footprint = self.create_client(Empty, '/change_footprint')

        while not self.service_change_footprint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /change_footprint not available, waiting again...')

    def change_footprint(self, new_footprint):
        request = Empty.Request()
        # Create a PolygonStamped message to represent the new footprint
        polygon = PolygonStamped()
        polygon.header.frame_id = "base_footprint"  # Set the frame id
        for point in new_footprint:
            pt = Point32()
            pt.x = point[0]
            pt.y = point[1]
            polygon.polygon.points.append(pt)
        # Publish the new footprint
        self.publish_footprint(polygon)
        # Call the service to change the footprint
        future = self.service_change_footprint.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Successfully changed footprint')
        else:
            self.get_logger().info('Failed to change footprint')

    def publish_footprint(self, footprint):
        # Publish the new footprint
        self.footprint_pub.publish(footprint)

def main(args=None):
    rclpy.init(args=args)
    node = ChangeFootprint()
    # Define the new footprint as a list of points (x, y)
    new_footprint = [[0.5, 0.4], [0.5, -0.4], [-0.5, -0.4], [-0.5, 0.4]]
    node.change_footprint(new_footprint)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
