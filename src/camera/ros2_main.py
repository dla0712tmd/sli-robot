import socket
import cv2
import numpy as np
import json
from ultralytics import YOLO
import supervision as sv
import threading

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import String

def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf:
            return None
        buf += newbuf
        count -= len(newbuf)
    return buf

class YOLOClientNode(Node):
    def __init__(self):
        super().__init__('yolo_client_node')
        
        # ROS2 퍼블리셔 초기화
        self.detection_publisher_ = self.create_publisher(Detection2DArray, 'detection_topic', 10)

        # 모델 및 추적기 설정
        self.model = YOLO('yolov8n.pt')
        self.tracker = sv.ByteTrack()
        self.box_annotator = sv.BoundingBoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()

        # 서버 연결 스레드 시작
        self.server_ip = '192.168.219.106'
        self.server_port = 9999
        self.client_socket = None
        
        self.connect_to_server()

    def connect_to_server(self):
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.server_ip, self.server_port))
            self.get_logger().info(f"Connected to server at {self.server_ip}:{self.server_port}")

            # 프레임 수신 및 처리 스레드 시작
            self.receive_thread = threading.Thread(target=self.receive_frames)
            self.receive_thread.start()

        except Exception as e:
            self.get_logger().error(f"Failed to connect to server: {e}")

    def send_detection_data(self, detections):
        # 검출된 객체 정보를 JSON 형식으로 변환하여 전송
        detection_data = [{
            "class_id": int(det_class_id),  # 클래스 ID를 정수로 변환
            "bbox": det_bbox.tolist(),      # 바운딩 박스 좌표를 리스트로 변환
            "confidence": float(det_confidence)  # 신뢰도를 부동소수점 숫자로 변환
        } for det_bbox, det_class_id, det_confidence in zip(detections.xyxy, detections.class_id, detections.confidence)]
        data_str = json.dumps(detection_data)  # JSON 문자열로 인코딩
        
        data_size = len(data_str.encode('utf-8'))
        self.client_socket.sendall(str(data_size).encode('utf-8').ljust(16))
        self.client_socket.sendall(data_str.encode('utf-8'))  # 데이터 전송
    
    def receive_frames(self):
        while rclpy.ok():
            length = recvall(self.client_socket, 16)
            if not length:
                break
            frame_length = int(length.strip())

            frame_data = recvall(self.client_socket, frame_length)
            if not frame_data:
                break

            data = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)

            # 비동기적으로 프레임 처리
            annotated_frame, detections = self.process_frame(frame)

            # 결과 비디오 프레임 표시 및 검출 데이터 전송
            cv2.imshow('YOLOv8 Person Detection', annotated_frame)
            self.send_detection_data(detections)  # 검출 데이터 전송

            if cv2.waitKey(1) == ord('q'):
                break

        cv2.destroyAllWindows()

    def process_frame(self, frame):
        # YOLOv8 모델로 사람 탐지
        results = self.model(frame)
        if isinstance(results, list):
            results = results[0]

        detections = sv.Detections.from_ultralytics(results)
        detections = self.tracker.update_with_detections(detections)

        # 결과 프레임에 박스 및 레이블 추가
        annotated_frame = self.box_annotator.annotate(scene=frame.copy(), detections=detections)
        annotated_frame = self.label_annotator.annotate(scene=annotated_frame, detections=detections)

        # 검출 결과를 ROS2 메시지로 변환하여 퍼블리시
        self.publish_detections(detections)

        return annotated_frame, detections

    def publish_detections(self, detections):
        detection_msg = Detection2DArray()
        for det_bbox, det_class_id, det_confidence in zip(detections.xyxy, detections.class_id, detections.confidence):
            detection = Detection2D()
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = str(int(det_class_id))
            hypothesis.score = float(det_confidence)

            detection.results.append(hypothesis)
            detection.bbox.center.x = (det_bbox[0] + det_bbox[2]) / 2.0
            detection.bbox.center.y = (det_bbox[1] + det_bbox[3]) / 2.0
            detection.bbox.size_x = float(det_bbox[2] - det_bbox[0])
            detection.bbox.size_y = float(det_bbox[3] - det_bbox[1])

            detection_msg.detections.append(detection)

        self.detection_publisher_.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    yolo_client_node = YOLOClientNode()

    try:
        rclpy.spin(yolo_client_node)
    except KeyboardInterrupt:
        yolo_client_node.get_logger().info('Shutting down client node.')
    finally:
        if yolo_client_node.client_socket:
            yolo_client_node.client_socket.close()
        yolo_client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

