#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time

class AutonomousExcavator(Node):
    def __init__(self):
        super().__init__('autonomous_excavator')

        # Initialize serial connection to Pololu Mini Maestro
        try:
            self.serial_port = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
            self.get_logger().info("✅ Connected to Pololu Mini Maestro on /dev/ttyAMA0")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect to Pololu: {e}")
            raise SystemExit(1)

        # Define servo channels
        self.channel_bucket = 0
        self.channel_boom = 1
        self.channel_arm = 2
        self.channel_twist = 3
        self.channel_pump = 4
        self.channel_left_track = 5
        self.channel_right_track = 6

        # Servo values
        self.neutral = 6000
        self.pump_high = 8000

        # Initialize system
        self.initialize_channels()

        # Start the autonomous digging sequence
        self.run_autonomous_sequence()

    def set_target(self, channel, target):
        try:
            command = bytearray()
            command.append(0x84)  # Command byte
            command.append(channel)
            command.append(target & 0x7F)           # Low bits
            command.append((target >> 7) & 0x7F)    # High bits
            self.serial_port.write(command)
            self.get_logger().info(f"→ Set channel {channel} to {target}")
        except Exception as e:
            self.get_logger().error(f"❌ Error writing to channel {channel}: {e}")

    def initialize_channels(self):
        self.get_logger().info("🛠️ Initializing servos and arming pump ESC")

        # Step 1: Set pump to neutral to arm ESC
        self.set_target(self.channel_pump, self.neutral)
        time.sleep(2)

        # Step 2: Initialize all other actuators to neutral
        for ch in [self.channel_bucket, self.channel_boom, self.channel_arm,
                   self.channel_twist, self.channel_left_track, self.channel_right_track]:
            self.set_target(ch, self.neutral)
            time.sleep(0.5)

        # Step 3: Set pump to high value to activate hydraulics
        self.set_target(self.channel_pump, self.pump_high)
        self.get_logger().info("💧 Pump motor activated")

    def run_autonomous_sequence(self):
        self.get_logger().info("🕳️ Starting hole digging sequence (1x1 ft)")

        try:
            for row in range(2):  # Dig 2 rows (simulate moving forward)
                for scoop in range(4):  # 4 scoops per row (simulate 4 slices)

                    self.get_logger().info(f"🔁 Row {row+1}, Scoop {scoop+1}")

                    # Step 1: Lower the arm/bucket to ground
                    self.set_target(self.channel_arm, 5000)
                    self.set_target(self.channel_bucket, 4000)  # Tilt down
                    time.sleep(1)

                    # Step 2: Simulate scoop (twist)
                    self.set_target(self.channel_twist, 5500)
                    time.sleep(0.5)
                    self.set_target(self.channel_twist, 4500)
                    time.sleep(0.5)

                    # Step 3: Lift bucket with dirt
                    self.set_target(self.channel_arm, 7000)
                    self.set_target(self.channel_boom, 7000)
                    time.sleep(1)

                    # Step 4: Dump
                    self.set_target(self.channel_bucket, 8000)
                    time.sleep(1)

                    # Step 5: Reset position
                    self.set_target(self.channel_bucket, self.neutral)
                    self.set_target(self.channel_boom, self.neutral)
                    self.set_target(self.channel_arm, self.neutral)
                    time.sleep(1)

                # Move forward to next row
                self.get_logger().info("🚜 Advancing to next row")
                self.set_target(self.channel_left_track, 7000)
                self.set_target(self.channel_right_track, 7000)
                time.sleep(1)
                self.set_target(self.channel_left_track, self.neutral)
                self.set_target(self.channel_right_track, self.neutral)
                time.sleep(0.5)

            self.get_logger().info("✅ Hole digging sequence complete!")

        except KeyboardInterrupt:
            self.get_logger().info("⛔ Sequence interrupted by user")

    def destroy_node(self):
        self.get_logger().info("🧹 Shutting down node and resetting servos")
        self.set_target(self.channel_bucket, self.neutral)
        self.set_target(self.channel_boom, self.neutral)
        self.set_target(self.channel_arm, self.neutral)
        self.set_target(self.channel_twist, self.neutral)
        self.set_target(self.channel_pump, self.neutral)
        self.set_target(self.channel_left_track, self.neutral)
        self.set_target(self.channel_right_track, self.neutral)
        self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    autonomous_excavator = AutonomousExcavator()
    rclpy.spin(autonomous_excavator)
    autonomous_excavator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
