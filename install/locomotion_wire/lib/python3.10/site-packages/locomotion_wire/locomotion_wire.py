import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import sys
import termios
import tty
import time


# 공통 상수 정의
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_SPEED = 32
ADDR_MX_GOAL_POSITION = 30
PROTOCOL_VERSION = 1.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'  # 시스템에 맞게 조정
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
COMM_SUCCESS = 0

# 주행모드 모터 ID
DXL9_ID = 9 
DXL10_ID = 10  
DXL11_ID = 11 
DXL12_ID = 12
DXL13_ID = 13  
DXL14_ID = 14 
DXL15_ID = 15
DXL16_ID = 16  

# 중간관절 모터 ID
DXL19_ID = 19
DXL20_ID = 20  

# 스프링 및 와이어 모터 ID = 앞의 모터 번호 5,6,7,8
DXL5_ID = 5
DXL6_ID = 6
DXL7_ID = 7
DXL8_ID = 8

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_keyboard_controller')

        # 포트 핸들러 및 패킷 핸들러 초기화
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        

        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open the port")
            sys.exit(1)

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set the baudrate")
            sys.exit(1)

        # 주행모드 모터 토크 활성화
        for motor_id in [DXL9_ID, DXL10_ID, DXL11_ID, DXL12_ID, DXL13_ID, DXL14_ID, DXL15_ID, DXL16_ID]:
            self.enable_torque(motor_id)
        
        # 중간관절 모터 위치 설정 및 토크 활성화
        for motor_id in [DXL19_ID, DXL20_ID]:
            self.enable_torque(motor_id)
            self.set_motor_position(motor_id, 512)  # 위치를 512로 고정

        # 스프링 및 와이어 모터 토크 활성화
        for motor_id in [DXL5_ID, DXL6_ID, DXL7_ID, DXL8_ID]:
            self.enable_torque(motor_id)

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
                
                # 주행모드
                if key == '1':
                    for motor_id in [DXL9_ID, DXL10_ID, DXL11_ID, DXL12_ID, DXL13_ID, DXL14_ID, DXL15_ID, DXL16_ID]:
                        self.set_motor_speed(motor_id, 600)
                    print("주행모드 \n")

                # 반대방향
                elif key == '2':
                    for motor_id in [DXL9_ID, DXL10_ID, DXL11_ID, DXL12_ID, DXL13_ID, DXL14_ID, DXL15_ID, DXL16_ID]:
                        self.set_motor_speed(motor_id, 1624)
                    print("주행모드 반대방향\n")

                # 오른쪽 회전
                elif key == '3':
                    speeds = [600, 1624, 600, 1624, 600, 1624, 600, 1624]
                    for motor_id, speed in zip([DXL9_ID, DXL10_ID, DXL11_ID, DXL12_ID, DXL13_ID, DXL14_ID, DXL15_ID, DXL16_ID], speeds):
                        self.set_motor_speed(motor_id, speed)
                    print("다같은 방향으로 회전\n")

                # 왼쪽 회전
                elif key == '4':
                    speeds = [1624, 600, 1624, 600, 1624, 600, 1624, 600]
                    for motor_id, speed in zip([DXL9_ID, DXL10_ID, DXL11_ID, DXL12_ID, DXL13_ID, DXL14_ID, DXL15_ID, DXL16_ID], speeds):
                        self.set_motor_speed(motor_id, speed)
                    print("다 같이 반대방향으로 회전\n")

                # 스프링 및 와이어 모터 제어
                elif key == 'q':     # 위로 올리기 (7, 8번 당기기)
                    
                    self.set_motor_speed(DXL7_ID, 1824)
                    self.set_motor_speed(DXL8_ID, 800)
                    self.set_motor_speed(DXL5_ID, 160)
                    self.set_motor_speed(DXL6_ID, 1184)
                    print("q : 7,8번 당기기\n")
                    
                elif key == 'Q':    # 7, 8번 풀기
                    self.set_motor_speed(DXL7_ID, 800)
                    self.set_motor_speed(DXL8_ID, 1824)
                    self.set_motor_speed(DXL5_ID, 1184)
                    self.set_motor_speed(DXL6_ID, 160)
                    print("Q : 7,8번 풀기\n")
                    
                    
                elif key == 'w':    # 오른쪽 (6, 8 당기기)
                    self.set_motor_speed(DXL6_ID, 400)
                    self.set_motor_speed(DXL8_ID, 400)
                    self.set_motor_speed(DXL5_ID, 300)
                    self.set_motor_speed(DXL7_ID, 300)
                    print("w : 6,8번 당기기\n")
                    
                elif key == 'W':    # 오른쪽 풀기 (6, 8 풀기)
                    self.set_motor_speed(DXL6_ID, 1424)
                    self.set_motor_speed(DXL8_ID, 1424)
                    self.set_motor_speed(DXL5_ID, 1324)
                    self.set_motor_speed(DXL7_ID, 1324)
                    print("W : 6,8번 풀기\n")
                    
                elif key == 'e':    # 왼쪽 (5, 7 당기기)
                    self.set_motor_speed(DXL5_ID, 1424)
                    self.set_motor_speed(DXL7_ID, 1424)
                    self.set_motor_speed(DXL6_ID, 1324)
                    self.set_motor_speed(DXL8_ID, 1324)
                    print("e : 5,7 당기기\n")
                    
                elif key == 'E':    # 왼쪽 풀기 (5, 7 풀기)
                    self.set_motor_speed(DXL5_ID, 400)
                    self.set_motor_speed(DXL7_ID, 400)
                    self.set_motor_speed(DXL6_ID, 300)
                    self.set_motor_speed(DXL8_ID, 300)
                    print("E : 5,7 풀기\n")
                    
                elif key == 'r':    # 아래 당기기 (5, 6 당기기)
                    self.set_motor_speed(DXL5_ID, 1424)
                    self.set_motor_speed(DXL6_ID, 400)
                    self.set_motor_speed(DXL7_ID, 300)
                    self.set_motor_speed(DXL8_ID, 1324)
                    print("r : 5,6번 당기기\n")
                    
                elif key == 'R':    # 아래 풀기 (5, 6 풀기)
                    self.set_motor_speed(DXL5_ID, 400)
                    self.set_motor_speed(DXL6_ID, 1424)
                    self.set_motor_speed(DXL7_ID, 1324)
                    self.set_motor_speed(DXL8_ID, 300)
                    print("R : 5,6번 풀기\n")
                
                
                elif key == 'z':    # 싹 다 멈추기
                    self.set_motor_speed(DXL5_ID, 0)
                    self.set_motor_speed(DXL6_ID, 0)
                    self.set_motor_speed(DXL7_ID, 0)
                    self.set_motor_speed(DXL8_ID, 0)
                    print("5678 다 멈추기\n")
                
                    
                elif key == 'a':    # 7 당기기
                    self.set_motor_speed(DXL7_ID, 1624)
                    print("a : 7 당기기\n")
                    
                elif key == 'A':    # 7 풀기
                    self.set_motor_speed(DXL7_ID, 600)
                    print("A : 7 풀기\n")
                    
                elif key == 's':    # 8 당기기
                    self.set_motor_speed(DXL8_ID, 600)
                    print("s : 8 당기기\n")
                    
                elif key == 'S':    # 8 풀기
                    self.set_motor_speed(DXL8_ID, 1624)
                    print("S : 8 풀기\n")
                    
                elif key == 'd':    # 5 당기기
                    self.set_motor_speed(DXL5_ID, 600)
                    print("d : 5 당기기\n")
                    
                elif key == 'D':    # 5 풀기
                    self.set_motor_speed(DXL5_ID, 1624)
                    print("D : 5 풀기\n")
                    
                elif key == 'f':    # 6 당기기
                    self.set_motor_speed(DXL6_ID, 1624)
                    print("f : 6 당기기\n")
                    
                elif key == 'F':    # 6 풀기
                    self.set_motor_speed(DXL6_ID, 600)
                    print("F : 6 풀기\n")
                    
                # 모든 모터 정지 및 종료
                elif key == '5' :
                    #self.stop_gimbal()
                    break
                
                else:
                    # 모터 속도를 0으로 설정하여 멈추기
                    for motor_id in [DXL9_ID, DXL10_ID, DXL11_ID, DXL12_ID, DXL13_ID, DXL14_ID, DXL15_ID, DXL16_ID, DXL5_ID, DXL6_ID, DXL7_ID, DXL8_ID]:
                        self.set_motor_speed(motor_id, 0)

        finally:
            self.restore_terminal()
            # 모든 모터의 토크를 비활성화하고 포트를 닫음
            for motor_id in [DXL9_ID, DXL10_ID, DXL11_ID, DXL12_ID, DXL13_ID, DXL14_ID, DXL15_ID, DXL16_ID, DXL5_ID, DXL6_ID, DXL7_ID, DXL8_ID, DXL19_ID, DXL20_ID]:
                self.disable_torque(motor_id)
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

