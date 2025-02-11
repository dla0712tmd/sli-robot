import socket
import cv2
import numpy as np
import json
import threading
import rclpy #추가
from rclpy.node import Node
from vision_msgs.msg import Detection

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(Detection2D, 'detection_topic', 10)

        # Start the data receiving and video sending threads
        self.start_threads()

    def start_threads(self):
        self.server_thread = threading.Thread(target=self.start_server)
        self.server_thread.start()

    def start_server(self):
        SERVER_IP = '0.0.0.0'
        SERVER_PORT = 9999

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((SERVER_IP, SERVER_PORT))
        server_socket.listen(1)
        print(f"Server listening on {SERVER_IP}:{SERVER_PORT}")

        conn, addr = server_socket.accept()
        print(f"Connected to {addr}")

        cap = cv2.VideoCapture(0)

        video_thread = threading.Thread(target=self.send_video, args=(conn, cap))
        video_thread.start()

        data_thread = threading.Thread(target=self.receive_data, args=(conn,))
        data_thread.start()

        video_thread.join()
        data_thread.join()

        cap.release()
        conn.close()
        server_socket.close()


    def recvall(sock, count):
        buffer = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf:
                return None
            buffer += newbuf
            count -= len(newbuf)
        return buffer

    def calculate_distance_and_direction(bbox, image_dimensions):
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
    
        image_width, image_height = image_dimensions
        x_camera_center = image_width / 2
        y_camera_center = image_height / 2
    
        dx = center_x - x_camera_center
        dy = center_y - y_camera_center
    
        if dx > 0:
            horizontal_direction = 'right'
        elif dx < 0:
            horizontal_direction = 'left'
        else:
            horizontal_direction = 'center'
    
        if dy > 0:
            vertical_direction = 'down'
        elif dy < 0:
            vertical_direction = 'up'
        else:
            vertical_direction = 'center'
    
        width = x2 - x1
        height = y2 - y1
    
        rate_area = (width * height) / (image_width * image_height) * 100
        distance = np.sqrt(dx**2 + dy**2)
   
        return distance, rate_area, horizontal_direction, vertical_direction

    def send_video(conn, cap):
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("프레임을 읽을 수 없습니다.")
                    break

                encoded, buffer = cv2.imencode('.jpg', frame)
                data = np.array(buffer)
                stringData = data.tobytes()

                conn.sendall((str(len(stringData))).encode().ljust(16))
                conn.sendall(stringData)

        except Exception as e:
            print(f"비디오 송신 중 에러 발생: {e}")

    def receive_data(conn):
        image_width, image_height = 640, 480
        try:
            while True:
                data_size_info = recvall(conn, 16)
                if not data_size_info:
                    print("데이터 크기 정보 수신 실패")
                    break

                data_size = int(data_size_info.decode().strip())
                json_data = recvall(conn, data_size)
                if not json_data:
                    print("완전한 데이터를 수신하지 못했습니다.")
                    continue

                try:
                    detections = json.loads(json_data.decode('utf-8'))
                    for detection in detections:
                        rounded_bbox = [round(coord, 2) for coord in detection['bbox']]
                        rounded_confidence = round(detection.get('confidence', 0), 2)
                        distance, rate_area, horizontal_direction, vertical_direction = calculate_distance_and_direction(rounded_bbox, (image_width, image_height))
                    
                        print("바운딩 박스 좌표:", rounded_bbox)
                        print("클래스 ID:", detection.get('class_id'))
                        print("신뢰도:", rounded_confidence)
                        print("박스 비율:", rate_area)
                        print("x방향:", horizontal_direction)
                        print("y방향:", vertical_direction)

                except json.JSONDecodeError as e:
                    print("JSON 파싱 오류:", e)

        except Exception as e:
            print(f"데이터 수신 중 에러 발생: {e}")

def main():
    SERVER_IP = '0.0.0.0'
    SERVER_PORT = 9999

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((SERVER_IP, SERVER_PORT))
    server_socket.listen(1)
    print(f"서버가 {SERVER_IP}:{SERVER_PORT}에서 대기 중입니다.")

    conn, addr = server_socket.accept()
    print(f"{addr}와 연결되었습니다.")

    cap = cv2.VideoCapture(0)

    video_thread = threading.Thread(target=send_video, args=(conn, cap))
    video_thread.start()

    data_thread = threading.Thread(target=receive_data, args=(conn,))
    data_thread.start()

    video_thread.join()
    data_thread.join()

    cap.release()
    conn.close()
    server_socket.close()
    print("서버 연결이 종료되었습니다.")

if __name__ == '__main__':
    main()
