import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
from math import pi
import time

class ExcavatorIMUNode(Node):
    def __init__(self):
        super().__init__('excavator_imu_node')
        
        # Publisher for IMU data
        self.imu_publisher = self.create_publisher(
            Imu,
            'excavator/imu',
            10  # QoS profile depth
        )
        
        # Timer for publishing IMU data
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # 10Hz
        
        # Initialize IMU message
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'
        
        self.get_logger().info('Excavator IMU Node has been started')

    def publish_imu_data(self):
        
        # Replace with real IMU data. For now, this will generate dummy data for testing.
      
        current_time = self.get_clock().now().to_msg()
        self.imu_msg.header.stamp = current_time
        
        # In reality, this will read these values from the IMU sensor
        # For now, there will be dummy data
        self.imu_msg.orientation.x = 0.0
        self.imu_msg.orientation.y = 0.0
        self.imu_msg.orientation.z = 0.0
        self.imu_msg.orientation.w = 1.0
        
        # Angular velocity (rad/s)
        self.imu_msg.angular_velocity.x = 0.0
        self.imu_msg.angular_velocity.y = 0.0
        self.imu_msg.angular_velocity.z = 0.0
        
        # Linear acceleration (m/s^2)
        self.imu_msg.linear_acceleration.x = 0.0
        self.imu_msg.linear_acceleration.y = 0.0
        self.imu_msg.linear_acceleration.z = 9.81  # Gravity
        
        # Publish the message
        self.imu_publisher.publish(self.imu_msg)

def main(args=None):
    rclpy.init(args=args)
    
    imu_node = ExcavatorIMUNode()
    
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
