import socket
import cv2
import numpy as np
import json
from ultralytics import YOLO
import supervision as sv
import threading

def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf:
            return None
        buf += newbuf
        count -= len(newbuf)
    return buf

def send_detection_data(sock, detections):
    # 검출된 객체 정보를 JSON 형식으로 변환하여 전송
    detection_data = [{
        "class_id": int(det_class_id),  # 클래스 ID를 정수로 변환
        "bbox": det_bbox.tolist(),      # 바운딩 박스 좌표를 리스트로 변환
        "confidence": float(det_confidence)  # 신뢰도를 부동소수점 숫자로 변환
    } for det_bbox, det_class_id, det_confidence in zip(detections.xyxy, detections.class_id, detections.confidence)]
    data_str = json.dumps(detection_data)  # JSON 문자열로 인코딩
    
    data_size = len(data_str.encode('utf-8'))
    sock.sendall(str(data_size).encode('utf-8').ljust(16))
    sock.sendall(data_str.encode('utf-8'))  # 데이터 전송
    
        
def process_frame(frame, model, tracker, box_annotator, label_annotator):
    # YOLOv8 모델로 사람 탐지
    results = model(frame)
    if isinstance(results, list):
        results = results[0]

    detections = sv.Detections.from_ultralytics(results)
    detections = tracker.update_with_detections(detections)

    # 결과 프레임에 박스 및 레이블 추가
    annotated_frame = box_annotator.annotate(scene=frame.copy(), detections=detections)
    annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections)

    return annotated_frame, detections

def receive_frames(client_socket, model, tracker, box_annotator, label_annotator):
    while True:
        length = recvall(client_socket, 16)
        if not length:
            break
        frame_length = int(length.strip())

        frame_data = recvall(client_socket, frame_length)
        if not frame_data:
            break

        data = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)

        # 비동기적으로 프레임 처리
        annotated_frame, detections = process_frame(frame, model, tracker, box_annotator, label_annotator)

        # 결과 비디오 프레임 표시 및 검출 데이터 전송
        cv2.imshow('YOLOv8 Person Detection', annotated_frame)
        send_detection_data(client_socket, detections)  # 검출 데이터 전송

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()

def main():
    SERVER_IP = '192.168.0.130'
    SERVER_PORT = 9999

    model = YOLO('yolov8n.pt')
    tracker = sv.ByteTrack()
    box_annotator = sv.BoundingBoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_IP, SERVER_PORT))
    print(f"{SERVER_IP}:{SERVER_PORT}에 연결되었습니다.")

    # 프레임 수신 및 처리 스레드 시작
    receive_thread = threading.Thread(target=receive_frames, args=(client_socket, model, tracker, box_annotator, label_annotator))
    receive_thread.start()
    receive_thread.join()

    client_socket.close()
    print("연결이 종료되었습니다.")

if __name__ == '__main__':
    main()
