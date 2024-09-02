import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import sys
import termios
import tty

ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_SPEED = 32
PROTOCOL_VERSION = 1.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'  # Adjust this as per your system
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL1_ID = 8
DXL2_ID = 9
COMM_SUCCESS = 0

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_keyboard_controller')

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open the port")
            sys.exit(1)

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set the baudrate")
            sys.exit(1)

        self.enable_torque(DXL1_ID)
        self.enable_torque(DXL2_ID)

        self.get_logger().info("Ready to control Dynamixel motors with keyboard")

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

    def set_motor_speed(self, dxl_id, speed):
        result, error = self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_GOAL_SPEED, speed)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to set speed for ID {dxl_id}: {error}")
        else:
       	    self.get_logger().info(f"Speed set for ID {dxl_id}: {speed}")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                if key == 'w':
                    self.set_motor_speed(DXL1_ID, 1000)
                    self.set_motor_speed(DXL2_ID, 2024)
                    self.get_logger().info("w")
                elif key == 's':
                    self.set_motor_speed(DXL1_ID, 1224)
                    self.set_motor_speed(DXL2_ID, 200)
                    self.get_logger().info("s")
                elif key == 'a':
                    self.set_motor_speed(DXL1_ID, 200)
                    self.set_motor_speed(DXL2_ID, 200)
                    self.get_logger().info("a")
                elif key == 'd':
                    self.set_motor_speed(DXL1_ID, 1224)
                    self.set_motor_speed(DXL2_ID, 1224)
                    self.get_logger().info("d")
                elif key == 'q':
                    break
                else:
                    self.set_motor_speed(DXL1_ID, 0)
                    self.set_motor_speed(DXL2_ID, 0)
                   
        finally:
            self.disable_torque(DXL1_ID)
            self.disable_torque(DXL2_ID)
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

