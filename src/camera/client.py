import socket
import cv2
import numpy as np

def main():
    # 서버 설정
    SERVER_IP = '192.168.219.104'  # 라즈베리 파이의 실제 IP 주소로 변경하세요.
    SERVER_PORT = 9998                # 서버와 동일한 포트 번호

    # 소켓 생성 및 서버 연결
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_IP, SERVER_PORT))
    print(f"{SERVER_IP}:{SERVER_PORT}에 연결되었습니다.")

    try:
        while True:
            # 프레임 크기 수신
            length = recvall(client_socket, 16)
            if not length:
                break
            frame_length = int(length.strip())

            # 프레임 데이터 수신
            frame_data = recvall(client_socket, frame_length)
            if not frame_data:
                break

            # 프레임 디코딩 및 표시
            data = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
            cv2.imshow('Received Video', frame)

            if cv2.waitKey(1) == ord('q'):
                break

    except Exception as e:
        print(f"에러 발생: {e}")
    finally:
        client_socket.close()
        cv2.destroyAllWindows()
        print("연결이 종료되었습니다.")

def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf:
            return None
        buf += newbuf
        count -= len(newbuf)
    return buf

if __name__ == '__main__':
    main()
