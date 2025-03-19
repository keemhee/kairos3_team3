#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import pyzbar.pyzbar as pyzbar
import numpy as np
import time
import socket
import threading
from kairos_interfaces.srv import PackageInfo
from std_msgs.msg import Bool

mtx = np.loadtxt('/home/kairos/ws_project/src/QR_detector/QR_detector/mtx_2.csv', delimiter=',')
dist = np.loadtxt('/home/kairos/ws_project/src/QR_detector/QR_detector/dist_2.csv', delimiter=',')

SERVER_IP = "172.30.1.62"
SERVER_PORT = 10000

class Packageinfo():
    def __init__(self, packageid, region, width, depth, height, process):
        self.packageid = packageid
        self.region = region
        self.width = width
        self.depth = depth
        self.height = height
        self.process = process
    def __repr__(self):
        return f"Package=(packageid={self.packageid}, region={self.region}, width={self.width}, depth={self.depth}, height={self.height}, process={self.process})"

class Detect_Send(Node):
    def __init__(self):
        super().__init__("Detect_and_Send")
        self.cap = cv2.VideoCapture(0)
        self.width = 1280
        self.height = 720
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 수동 노출 모드
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -50)  # 노출 값 설정

        self.forces_x = 380
        self.forces_y = 600

        self.package_info_service = self.create_service(PackageInfo, "package_info", self.package_info_callback)
        self.package_delete_sub = self.create_subscription(Bool, "package_delete", self.package_delete_callback, 10)
        self.socket_client = SoketClient(self)

        self.cap_lock = threading.Lock()
        self.qrdata_lock = threading.Lock()
        self.package_info_lock = threading.Lock()

        self.qrdata = []
        self.package_info = []
        self.complated_package = []
        self.get_logger().info("Detect and Send Node is started...")

        self.camera_thread_running = True
        
        while True:
            if self.socket_client.receive_thread.is_alive() is True:
                self.camera_thread = threading.Thread(target=self.detect_from_camera)
                self.camera_thread.daemon = True
                self.camera_thread.start()
                break
            else:
                self.get_logger().info("Wait Receive Thread Start...")

    def package_delete_callback(self, msg):
        self.complated_package.clear()

    def package_info_callback(self, request, response):
        if not self.qrdata or not self.package_info:
            self.get_logger().info("QR data is not detected..")
            response.package_id = -2
            return response
        
        package = self.package_info.pop(0)
        package_id = self.qrdata.pop(0)
        self.complated_package.append(package_id)

        if package_id != package.packageid:
            self.get_logger().info("Package ID is not matched..")
            response.package_id = -1
            return response
        
        self.get_logger().info("received service package info")
        response.package_id = package.packageid
        response.region = package.region
        response.width = package.width
        response.depth = package.depth
        response.height = package.height
        response.process = package.process

        #package position get
        center, theta = self.get_package_position(package_id)
        response.x = center[0]
        response.y = center[1]
        response.theta = theta
        self.get_logger().info("service response: " + str(response))
        return response
    
    def get_package_position(self, package_id):
        while True:
            with self.cap_lock:
                ret, frame = self.cap.read()
            frame = cv2.flip(frame,0)
            frame = cv2.flip(frame,1)
            frame = cv2.undistort(frame, mtx, dist)

            # 프레임 중앙 부분만 자르기
            height, width = frame.shape[:2]
            # 원하는 중앙 영역의 너비 설정 (예: 전체 너비의 50%)
            center_width = int(width * 0.5)
            # 중앙 좌표 계산
            center_x = width // 2
            # 잘라낼 영역의 시작점과 끝점 계산
            start_x = center_x - (center_width // 2)
            end_x = center_x + (center_width // 2)
            # 프레임 자르기
            frame = frame[:, start_x:end_x]

            result_center = None
            result_angle = None

            decodedObjects = pyzbar.decode(frame)
            if decodedObjects:
                for obj in decodedObjects:
                    if package_id != int(obj.data.decode('utf-8')): 
                        continue
                    points = obj.polygon
                    # 포인트를 numpy 배열로 변환
                    points_array = np.array([point for point in points], dtype=np.float32)
                    # 최소 외접 사각형 계산
                    rect = cv2.minAreaRect(points_array)
                    # rect는 ((center_x, center_y), (width, height), angle) 형태로 반환됨
                    center, dimensions, angle = rect
                    # 각도가 -90°에서 0° 사이일 경우, 너비와 높이를 교환하고 각도를 조정
                    # if angle < -45:
                    #     width, height = dimensions
                    #     angle += 90
                    # else:
                    #     width, height = dimensions

                    result_center = (center[0], center[1])
                    result_angle = round(angle, 2)
                
                if result_center is not None:
                    break

        return result_center, result_angle 

    def detect_from_camera(self):
        global mtx, dist
        while self.camera_thread_running:
            with self.cap_lock:
                ret, frame = self.cap.read()
            if not ret:
                self.get_logger().info("Camera error..")
                break
            frame = cv2.flip(frame,0)
            frame = cv2.flip(frame,1)
            frame = cv2.undistort(frame, mtx, dist)

            # 프레임 중앙 부분만 자르기
            height, width = frame.shape[:2]
            # 원하는 중앙 영역의 너비 설정 (예: 전체 너비의 50%)
            center_width = int(width * 0.5)
            # 중앙 좌표 계산
            center_x = width // 2
            # 잘라낼 영역의 시작점과 끝점 계산
            start_x = center_x - (center_width // 2)
            end_x = center_x + (center_width // 2)
            # 프레임 자르기
            frame = frame[:, start_x:end_x]

            decodedObjects = pyzbar.decode(frame)
            if decodedObjects:
                for obj in decodedObjects:
                    points = obj.polygon
                    # 포인트를 numpy 배열로 변환
                    points_array = np.array([point for point in points], dtype=np.float32)
                    # 최소 외접 사각형 계산
                    rect = cv2.minAreaRect(points_array)
                    # rect는 ((center_x, center_y), (width, height), angle) 형태로 반환됨
                    center, dimensions, angle = rect
                    # 각도가 -90°에서 0° 사이일 경우, 너비와 높이를 교환하고 각도를 조정
                    # if angle < -45:
                    #     width, height = dimensions
                    #     angle += 90
                    # else:
                    #     width, height = dimensions

                    center = (int(center[0]), int(center[1]))
                    # 최소 외접 사각형 시각화
                    # box = cv2.boxPoints(rect)
                    # box = np.int0(box)
                    # cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                    
                    # 중심점 표시
                    cv2.circle(frame, center, 2, (0, 255, 0), -1)
                    
                    # 화면 중심점과 QR 코드 중심점 사이의 거리 계산
                    dx = center[0] - self.forces_x
                    dy = center[1] - self.forces_y
                    cv2.putText(frame, f"({dx}, {dy}, {int(angle)})", (center[0] + 10, center[1] - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                    
                    #cv2.putText(frame, f"Center: {center[0], center[1]}", (center[0] + 10, center[1] + 10), 
                    #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                    
                    qrdata = int(obj.data.decode('utf-8'))
                    if qrdata in self.qrdata or qrdata in self.complated_package:
                        continue
                    with self.qrdata_lock:
                        self.qrdata.append(qrdata)
                    self.get_logger().info(f'QR data: {qrdata}')
                    self.socket_client.send_message(f"{qrdata}\n")

            cv2.line(frame, (self.forces_x, 0), (self.forces_x, self.height), (255, 0, 0), 1)
            cv2.line(frame, (0, self.forces_y), (self.width, self.forces_y), (255, 0, 0), 1)

            cv2.imshow('frame', frame)
            cv2.waitKey(1)
            
            time.sleep(0.03)
    
    def destroy_node(self):
        self.socket_client.running = False
        if self.socket_client.receive_thread.is_alive():
            self.socket_client.receive_thread.join()

        self.camera_thread_running = False
        if self.camera_thread.is_alive():
            self.camera_thread.join()
        super().destroy_node()
        self.cap.release()
        cv2.destroyAllWindows()

class SoketClient():
    def __init__(self, qr_node):
        # 소켓 초기화
        self.client = None
        self.running = True
        self.qr_node = qr_node

        self.ip_address = SERVER_IP
        self.port = SERVER_PORT

        self.connect_to_server()

        # 수신 스레드 시작
        self.receive_thread = threading.Thread(target=self.receive_messages)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def connect_to_server(self):
        """서버에 연결"""
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client.connect((self.ip_address, self.port))
            self.qr_node.get_logger().info(f"{self.ip_address} Connect Server..")
        except ConnectionRefusedError:
            #서버 연결 실패 로그
            self.qr_node.get_logger().info(f"{self.ip_address} Connect Fail..")
            self.running = False

    def receive_messages(self):
        while self.running:
            try:
                response = self.client.recv(1024).decode('utf-8')
                if not response:
                    self.qr_node.get_logger().info("disconnect server")
                    break

                # 🔹 데이터 파싱
                package = self.parse_server_data(response)
                if package:
                    self.qr_node.get_logger().info(f"Package: {package}")
                    with self.qr_node.package_info_lock:
                        self.qr_node.package_info.append(package)

            except ConnectionResetError:
                self.qr_node.get_logger().info("disconnect server")
                break

    def send_message(self, message):
        """서버에 메시지 전송"""
        try:
            self.client.sendall(message.encode('utf-8'))
        except Exception as e:
            self.qr_node.get_logger().info(f"send message error: {e}")

    def parse_server_data(self, response):
        """Parses the server response into a `Packageinfo` object"""
        try:
            parts = response.split(", ")
            data = {}

            # Parse each "key: value" pair
            for part in parts:
                key_value = part.split(": ")
                if len(key_value) == 2:
                    key, value = key_value
                    data[key.strip()] = value.strip()  # Store as key-value pair

            # Convert extracted values correctly
            return Packageinfo(
                packageid=int(data.get("packageID", "")),
                region=data.get("Region", ""),
                width=float(data.get("width", 0.0)),  # 🔹 Convert to float
                depth=float(data.get("depth", 0.0)),  # 🔹 Convert to float
                height=float(data.get("height", 0.0)),  # 🔹 Convert to float
                process=int(data.get("process", ""))
            )

        except ValueError as e:
            print(f"Data conversion error: {e}")
            return None

        except Exception as e:
            print(f"Parsing error: {e}")
            return None

def main(args=None):
    rclpy.init(args = args)
    qr_node = Detect_Send()
    try:
        rclpy.spin(qr_node)
    except KeyboardInterrupt:
        qr_node.get_logger().info("KeyboardInterrupt Stop.")
    finally:
        qr_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()