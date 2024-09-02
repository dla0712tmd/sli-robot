# locomotion_wire/locomotion.py
# 주행모드 locomotion
# 사용하는 모터 머리 부터 순서대로 9, 10, 12, 11, 13, 14, 15, 16
# 중간관절 고정 필요  중간관절 모터 19, 20


import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import sys
import termios
import tty
import time

# 상수 정의
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_GOAL_SPEED = 32
PROTOCOL_VERSION = 1.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'  # 시스템에 맞게 조정 필요
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL9_ID = 9 
DXL10_ID = 10  
DXL11_ID = 11 
DXL12_ID = 12
DXL13_ID = 13  
DXL14_ID = 14 
DXL15_ID = 15
DXL16_ID = 16  
DXL19_ID = 19
DXL20_ID = 20  

COMM_SUCCESS = 0

class LocomotionController(Node):
    def __init__(self):
        super().__init__('locomotion_controller')

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open the port")
            sys.exit(1)

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set the baudrate")
            sys.exit(1)

        # 각 모터에 대해 토크 활성화
        for motor_id in [DXL9_ID, DXL10_ID, DXL11_ID, DXL12_ID, DXL13_ID, DXL14_ID, DXL15_ID, DXL16_ID]:
            self.enable_torque(motor_id)

        # 중간관절 모터 19, 20은 포지션 모드로 고정
        self.enable_torque(DXL19_ID)
        self.enable_torque(DXL20_ID)
        self.set_motor_position(DXL19_ID, 512)  # Position fixed to 512
        self.set_motor_position(DXL20_ID, 512)  # Position fixed to 512

        self.get_logger().info("Ready to control Dynamixel motors with keyboard")
        self.old_termios = termios.tcgetattr(sys.stdin)

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_termios)

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
            self.get_logger().error(f"Failed to set speed for ID {dxl_id}: {error}\n")
        else:
            self.get_logger().info(f"Speed set for ID {dxl_id}: {speed}\n")
        time.sleep(0.01)  # 로그 간 시간 차로 간격 조정

    def set_motor_position(self, dxl_id, position):
        result, error = self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_GOAL_POSITION, position)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to set position for ID {dxl_id}: {error}")
        else:
            self.get_logger().info(f"Position set for ID {dxl_id}: {position}")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                if key == '1':  # 주행모드
                    self.set_motor_speed(DXL9_ID, 600)
                    self.set_motor_speed(DXL10_ID, 600)
                    self.set_motor_speed(DXL11_ID, 600)
                    self.set_motor_speed(DXL12_ID, 600)
                    self.set_motor_speed(DXL13_ID, 600)
                    self.set_motor_speed(DXL14_ID, 600)
                    self.set_motor_speed(DXL15_ID, 600)
                    self.set_motor_speed(DXL16_ID, 600)
                    print("주행모드 \n")

                elif key == '2':  # 반대방향
                    self.set_motor_speed(DXL9_ID, 1624)
                    self.set_motor_speed(DXL10_ID, 1624)
                    self.set_motor_speed(DXL11_ID, 1624)
                    self.set_motor_speed(DXL12_ID, 1624)
                    self.set_motor_speed(DXL13_ID, 1624)
                    self.set_motor_speed(DXL14_ID, 1624)
                    self.set_motor_speed(DXL15_ID, 1624)
                    self.set_motor_speed(DXL16_ID, 1624)
                    print("주행모드 반대방향\n")

                elif key == '3':  # 오른쪽
                    self.set_motor_speed(DXL9_ID, 600)
                    self.set_motor_speed(DXL10_ID, 1624)
                    self.set_motor_speed(DXL11_ID, 600)
                    self.set_motor_speed(DXL12_ID, 1624)
                    self.set_motor_speed(DXL13_ID, 600)
                    self.set_motor_speed(DXL14_ID, 1624)
                    self.set_motor_speed(DXL15_ID, 600)
                    self.set_motor_speed(DXL16_ID, 1624)
                    print("다같은 방향으로 회전\n")

                elif key == '4':  # 왼쪽
                    self.set_motor_speed(DXL9_ID, 1624)
                    self.set_motor_speed(DXL10_ID, 600)
                    self.set_motor_speed(DXL11_ID, 1624)
                    self.set_motor_speed(DXL12_ID, 600)
                    self.set_motor_speed(DXL13_ID, 1624)
                    self.set_motor_speed(DXL14_ID, 600)
                    self.set_motor_speed(DXL15_ID, 1624)
                    self.set_motor_speed(DXL16_ID, 600)
                    print("다같은 방향으로 회전\n")

                elif key == '5':  # 정지
                    break

                else:  # 모터 정지
                    self.set_motor_speed(DXL9_ID, 0)
                    self.set_motor_speed(DXL10_ID, 0)
                    self.set_motor_speed(DXL11_ID, 0)
                    self.set_motor_speed(DXL12_ID, 0)
                    self.set_motor_speed(DXL13_ID, 0)
                    self.set_motor_speed(DXL14_ID, 0)
                    self.set_motor_speed(DXL15_ID, 0)
                    self.set_motor_speed(DXL16_ID, 0)

        finally:
            self.restore_terminal()
            for motor_id in [DXL9_ID, DXL10_ID, DXL11_ID, DXL12_ID, DXL13_ID, DXL14_ID, DXL15_ID, DXL16_ID]:
                self.disable_torque(motor_id)
            self.port_handler.closePort()
            print("종료")

def main(args=None):
    rclpy.init(args=args)
    locomotion_controller = LocomotionController()
    locomotion_controller.run()
    rclpy.shutdown()

