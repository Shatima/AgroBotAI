
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.moist_pub = self.create_publisher(Float32, 'sensor/moisture', 10)
        self.timer = self.create_timer(5.0, self.read_sensors)

    def read_sensors(self):
        # Dummy values; replace with actual sensor code
        moisture = 40.0
        msg = Float32()
        msg.data = moisture
        self.moist_pub.publish(msg)
        self.get_logger().info(f'Published Moisture: {moisture}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
