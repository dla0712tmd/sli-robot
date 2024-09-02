# locomotion_wire/wire.py
#스프링 및 와이어 모터 제어 
#모터 1,2,3,4
#뒷 모터 5,6,7,8

# 머리
# 5  6
# 7  8

# ------

# 1  2
# 3  4




import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import sys
import termios
import tty
import time

# Dynamixel 제어 주소 및 설정
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_SPEED = 32
PROTOCOL_VERSION = 1.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'  # 시스템에 맞게 조정 필요
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
COMM_SUCCESS = 0

# Dynamixel 모터 ID (머리를 앞쪽으로 본 상태에서 설정)
DXL1_ID = 1  # 아래 왼쪽
DXL2_ID = 2  # 아래 오른쪽
DXL3_ID = 3  # 위 왼쪽
DXL4_ID = 4  # 위 오른쪽

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_keyboard_controller')

        # Dynamixel Port 및 Packet Handler 초기화
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open the port")
            sys.exit(1)

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set the baudrate")
            sys.exit(1)

        # 모든 모터에 대한 토크 활성화
        self.enable_torque(DXL1_ID)
        self.enable_torque(DXL2_ID)
        self.enable_torque(DXL3_ID)
        self.enable_torque(DXL4_ID)

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

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()

                # 모터 제어 키 매핑
                if key == 'q':  # 위로 올리기 (3,4번 당기기)
                    self.set_motor_speed(DXL3_ID, 1424)
                    self.set_motor_speed(DXL4_ID, 400)
                    self.set_motor_speed(DXL1_ID, 300)
                    self.set_motor_speed(DXL2_ID, 1324)
                    print("q : 3,4번 당기기\n")
                elif key == 'Q':  # (3,4번 풀기)
                    self.set_motor_speed(DXL3_ID, 400)
                    self.set_motor_speed(DXL4_ID, 1424)
                    self.set_motor_speed(DXL1_ID, 1324)
                    self.set_motor_speed(DXL2_ID, 300)
                    print("Q: 3,4번 풀기\n")
                elif key == 'w':  # 오른쪽 (2,4 당기기)
                    self.set_motor_speed(DXL2_ID, 400)
                    self.set_motor_speed(DXL4_ID, 400)
                    self.set_motor_speed(DXL1_ID, 300)
                    self.set_motor_speed(DXL3_ID, 300)
                    print("w: 2,4번 당기기\n")
                elif key == 'W':  # 오른쪽 풀기 (2,4 풀기)
                    self.set_motor_speed(DXL2_ID, 1424)
                    self.set_motor_speed(DXL4_ID, 1424)
                    self.set_motor_speed(DXL1_ID, 1324)
                    self.set_motor_speed(DXL3_ID, 1324)
                    print("W: 2,4번 풀기\n")
                # 여기에 추가적인 키 매핑 코드 작성...

                elif key == 'z':  # 프로그램 종료
                    break
                else:  # 멈추기
                    self.set_motor_speed(DXL1_ID, 0)
                    self.set_motor_speed(DXL2_ID, 0)
                    self.set_motor_speed(DXL3_ID, 0)
                    self.set_motor_speed(DXL4_ID, 0)

        finally:
            self.restore_terminal()
            self.disable_torque(DXL1_ID)
            self.disable_torque(DXL2_ID)
            self.disable_torque(DXL3_ID)
            self.disable_torque(DXL4_ID)
            self.port_handler.closePort()

def main(args=None):
    rclpy.init(args=args)
    controller = DynamixelController()

    try:
        controller.run()
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        controller.port_handler.closePort()

if __name__ == '__main__':
    main()

