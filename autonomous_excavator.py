#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
import serial
import time


class AutonomousExcavator(Node):
    def __init__(self):
        super().__init__('autonomous_excavator')
        
        # Initialize serial connection to Pololu Mini Maestro
        self.serial_port = serial.Serial('/dev/ttyAMA0', 9600)  # Using ttyAMA0
        self.get_logger().info("Connected to Pololu Mini Maestro on /dev/ttyAMA0")


        # Define servo channels
        self.channel_bucket = 0
        self.channel_boom = 1
        self.channel_arm = 2
        self.channel_twist = 3
        self.channel_pump = 4
        self.channel_left_track = 5
        self.channel_right_track = 6


        # Neutral position (6000 in quarter-microseconds)
        self.neutral = 6000


        # High value for the pump motor (e.g., 8000)
        self.pump_high = 8000


        # Set all channels to neutral with a delay between each
        self.initialize_channels()


        # Run the autonomous sequence
        self.run_autonomous_sequence()


    def set_target(self, channel, target):
        """
        Send a target position to a specific channel on the Pololu Mini Maestro.
        Target is in quarter-microseconds (4000-8000).
        """
        command = bytearray()
        command.append(0x84)  # Command byte
        command.append(channel)  # Channel number
        command.append(target & 0x7F)  # Target low bits
        command.append((target >> 7) & 0x7F)  # Target high bits
        self.serial_port.write(command)


    def initialize_channels(self):
        """
        Set all channels to the neutral position (6000) with a delay between each.
        Set the pump channel to a high value (8000) to enable hydraulic movement.
        """
        self.get_logger().info("Setting all channels to neutral position")


        # Set each channel to neutral with a delay
        self.set_target(self.channel_bucket, self.neutral)
        time.sleep(1)  # 1-second delay
        self.set_target(self.channel_boom, self.neutral)
        time.sleep(1)
        self.set_target(self.channel_arm, self.neutral)
        time.sleep(1)
        self.set_target(self.channel_twist, self.neutral)
        time.sleep(1)
        self.set_target(self.channel_left_track, self.neutral)
        time.sleep(1)
        self.set_target(self.channel_right_track, self.neutral)
        time.sleep(1)


        # Set the pump channel to a high value (8000) to enable hydraulic movement
        self.set_target(self.channel_pump, self.pump_high)
        self.get_logger().info("Pump channel set to high value (8000) for hydraulic movement")


    def run_autonomous_sequence(self):
        """
        Autonomous sequence to control the excavator.
        """
        self.get_logger().info("Starting autonomous sequence")


        try:
            while rclpy.ok():
                # Keep the pump channel at a high value continuously
                self.set_target(self.channel_pump, self.pump_high)


                # Scoop sequence
                self.get_logger().info("Scooping...")
                self.set_target(self.channel_bucket, 4000)  # Lower bucket
                self.set_target(self.channel_boom, 7000)  # Raise boom
                self.set_target(self.channel_arm, 5000)  # Lower arm
                time.sleep(2)


                # Dump sequence
                self.get_logger().info("Dumping...")
                self.set_target(self.channel_bucket, 8000)  # Raise bucket
                self.set_target(self.channel_boom, 5000)  # Lower boom
                self.set_target(self.channel_arm, 7000)  # Raise arm
                time.sleep(2)


                # Return to neutral (except pump)
                self.get_logger().info("Returning to neutral position")
                self.set_target(self.channel_bucket, self.neutral)
                self.set_target(self.channel_boom, self.neutral)
                self.set_target(self.channel_arm, self.neutral)
                time.sleep(2)


        except KeyboardInterrupt:
            self.get_logger().info("Autonomous sequence interrupted")


    def destroy_node(self):
        """
        Cleanup on node shutdown.
        """
        # Set all channels to neutral before shutting down
        self.set_target(self.channel_bucket, self.neutral)
        self.set_target(self.channel_boom, self.neutral)
        self.set_target(self.channel_arm, self.neutral)
        self.set_target(self.channel_twist, self.neutral)
        self.set_target(self.channel_pump, self.neutral)  # Set pump to neutral on shutdown
        self.set_target(self.channel_left_track, self.neutral)
        self.set_target(self.channel_right_track, self.neutral)


        self.serial_port.close()
        self.get_logger().info("Serial connection closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    autonomous_excavator = AutonomousExcavator()
    rclpy.spin(autonomous_excavator)
    autonomous_excavator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
