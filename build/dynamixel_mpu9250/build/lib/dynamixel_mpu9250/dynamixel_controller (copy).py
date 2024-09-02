#짐벌 (mpu9250 + ax-12a 포지션 모드)

import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import math
import smbus
import time

# MPU9250 I2C address and registers
ADDR_MPU9250 = 0x68
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
ACCEL_XOUT_H = 0x3b
ACCEL_YOUT_H = 0x3d
ACCEL_ZOUT_H = 0x3f

# Dynamixel MX Series register addresses
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30
PROTOCOL_VERSION = 1.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL17_ID = 17
DXL18_ID = 18
COMM_SUCCESS = 0

# Function to read from MPU9250
def read_word_2c(bus, adr):
    high = bus.read_byte_data(ADDR_MPU9250, adr)
    low = bus.read_byte_data(ADDR_MPU9250, adr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -(65536 - val)
    else:
        return val

def dist(a, b):
    return math.sqrt(a * a + b * b)

def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)

def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_position_controller')

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.bus = smbus.SMBus(1)  # Initialize I2C bus

        # Open port and set baud rate
        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open the port")
            exit(1)

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set the baudrate")
            exit(1)

        # Enable torque
        self.enable_torque(DXL17_ID)
        self.enable_torque(DXL18_ID)

        # Wake up MPU9250
        self.bus.write_byte_data(ADDR_MPU9250, power_mgmt_1, 0)

        self.get_logger().info("Ready to control Dynamixel motors based on MPU9250")

    def enable_torque(self, dxl_id):
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to enable torque for ID {dxl_id}: {error}")
        else:
            self.get_logger().info(f"Torque enabled for ID {dxl_id}")

    def disable_torque(self, dxl_id):
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to disable torque for ID {dxl_id}: {error}")
        else:
            self.get_logger().info(f"Torque disabled for ID {dxl_id}")

    def set_motor_position(self, dxl_id, position):
        # Position range for MX series: 0 to 1023
        position = int(max(0, min(1023, position)))
        result, error = self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_GOAL_POSITION, position)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to set position for ID {dxl_id}: {error}")
        else:
            self.get_logger().info(f"Position set for ID {dxl_id}: {position}")

    def read_mpu9250_angles(self):
        accel_x = read_word_2c(self.bus, ACCEL_XOUT_H)
        accel_y = read_word_2c(self.bus, ACCEL_YOUT_H)
        accel_z = read_word_2c(self.bus, ACCEL_ZOUT_H)
        x_rot = get_x_rotation(accel_x, accel_y, accel_z)
        y_rot = get_y_rotation(accel_x, accel_y, accel_z)
        return x_rot, y_rot

    def run(self):
        try:
            while True:
                x_rot, y_rot = self.read_mpu9250_angles()
                self.get_logger().info(f"x_rotation: {x_rot:.2f}, y_rotation: {y_rot:.2f}")

                # Set positions based on the angle values
                self.set_motor_position(DXL17_ID, int(512 + x_rot))  # Example scaling and offset
                self.set_motor_position(DXL18_ID, int(512 - y_rot))  # Example scaling and offset

                time.sleep(0.1)

        finally:
            self.disable_torque(DXL17_ID)
            self.disable_torque(DXL18_ID)
            self.port_handler.closePort()

def main(args=None):
    rclpy.init(args=args)

    try:
        controller = DynamixelController()
        controller.run()
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        controller.port_handler.closePort()

if __name__ == '__main__':
    main()
