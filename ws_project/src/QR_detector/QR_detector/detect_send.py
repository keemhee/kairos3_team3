#!/usr/bin/env python3  # Python 3 인터프리터를 사용하도록 지정
import rclpy  # ROS 2의 Python 클라이언트 라이브러리를 가져옴
from rclpy.node import Node  # ROS 2 노드 클래스를 가져옴
import cv2  # OpenCV 라이브러리를 가져옴 (이미지 처리용)
import pyzbar.pyzbar as pyzbar  # QR 코드 인식을 위한 pyzbar 모듈을 가져옴
import numpy as np  # 수학적 계산 및 배열 처리를 위한 NumPy 라이브러리를 가져옴
import time  # 시간 관련 기능을 사용하기 위해 가져옴
import socket  # 소켓 통신을 위해 가져옴
import threading  # 멀티스레딩을 위해 가져옴
from kairos_interfaces.srv import PackageInfo  # 사용자 정의 ROS 2 서비스 인터페이스를 가져옴
from std_msgs.msg import Bool  # ROS 2 표준 메시지 타입 Bool을 가져옴

mtx = np.loadtxt('/home/kairos/ws_project/src/QR_detector/QR_detector/mtx_2.csv', delimiter=',')  # 카메라 캘리브레이션 행렬을 CSV 파일에서 불러옴
dist = np.loadtxt('/home/kairos/ws_project/src/QR_detector/QR_detector/dist_2.csv', delimiter=',')  # 카메라 왜곡 계수를 CSV 파일에서 불러옴

SERVER_IP = "172.30.1.62"  # 서버 IP 주소를 상수로 정의
SERVER_PORT = 10000  # 서버 포트 번호를 상수로 정의

class Packageinfo():  # 패키지 정보를 저장하기 위한 클래스 정의
    def __init__(self, packageid, region, width, depth, height, process):  # 생성자 정의, 패키지 속성 초기화
        self.packageid = packageid  # 패키지 ID 속성 설정
        self.region = region  # 패키지 지역 속성 설정
        self.width = width  # 패키지 너비 속성 설정
        self.depth = depth  # 패키지 깊이 속성 설정
        self.height = height  # 패키지 높이 속성 설정
        self.process = process  # 패키지 처리 상태 속성 설정
    def __repr__(self):  # 객체를 문자열로 표현하는 메서드 정의
        return f"Package=(packageid={self.packageid}, region={self.region}, width={self.width}, depth={self.depth}, height={self.height}, process={self.process})"  # 패키지 정보를 문자열로 반환

class Detect_Send(Node):  # ROS 2 노드를 상속받아 QR 코드 감지 및 전송 클래스 정의
    def __init__(self):  # 노드 초기화 메서드 정의
        super().__init__("Detect_and_Send")  # 부모 클래스(Node)의 생성자를 호출하며 노드 이름 설정
        self.cap = cv2.VideoCapture(0)  # 기본 카메라(인덱스 0)를 열어 비디오 캡처 객체 생성
        self.width = 1280  # 카메라 프레임 너비를 1280으로 설정
        self.height = 720  # 카메라 프레임 높이를 720으로 설정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)  # 카메라 프레임 너비 설정 적용
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)  # 카메라 프레임 높이 설정 적용

        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 카메라 자동 노출을 수동 모드로 설정 (0.25는 수동 모드 활성화)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -50)  # 카메라 노출 값을 -50으로 설정 (어두운 환경 조정)

        self.forces_x = 380  # 프레임에서 강제 중심 X 좌표 정의 (화면 중앙 기준)
        self.forces_y = 600  # 프레임에서 강제 중심 Y 좌표 정의 (화면 중앙 기준)

        self.package_info_service = self.create_service(PackageInfo, "package_info", self.package_info_callback)  # 패키지 정보 서비스 생성
        self.package_delete_sub = self.create_subscription(Bool, "package_delete", self.package_delete_callback, 10)  # 패키지 삭제 토픽 구독 설정
        self.socket_client = SoketClient(self)  # 소켓 클라이언트 객체 생성 및 노드 전달

        self.cap_lock = threading.Lock()  # 카메라 접근을 위한 락 객체 생성 (스레드 안전성 보장)
        self.qrdata_lock = threading.Lock()  # QR 데이터 접근을 위한 락 객체 생성 (스레드 안전성 보장)
        self.package_info_lock = threading.Lock()  # 패키지 정보 접근을 위한 락 객체 생성 (스레드 안전성 보장)

        self.qrdata = []  # 감지된 QR 코드 데이터를 저장할 리스트 초기화
        self.package_info = []  # 패키지 정보를 저장할 리스트 초기화
        self.complated_package = []  # 완료된 패키지 ID를 저장할 리스트 초기화
        self.get_logger().info("Detect and Send Node is started...")  # 노드 시작 로그 출력

        self.camera_thread_running = True  # 카메라 스레드 실행 상태 플래그 초기화
        
        while True:  # 무한 루프 시작 (소켓 수신 스레드가 시작될 때까지 대기)
            if self.socket_client.receive_thread.is_alive() is True:  # 소켓 수신 스레드가 살아있는지 확인
                self.camera_thread = threading.Thread(target=self.detect_from_camera)  # 카메라 감지 스레드 생성
                self.camera_thread.daemon = True  # 스레드를 데몬 모드로 설정 (메인 종료 시 함께 종료)
                self.camera_thread.start()  # 카메라 감지 스레드 시작
                break  # 소켓 연결 확인 후 루프 종료
            else:  # 소켓 수신 스레드가 아직 시작되지 않음
                self.get_logger().info("Wait Receive Thread Start...")  # 대기 메시지 로그 출력

    def package_delete_callback(self, msg):  # 패키지 삭제 토픽 콜백 함수 정의
        self.complated_package.clear()  # 완료된 패키지 리스트 초기화

    def package_info_callback(self, request, response):  # 패키지 정보 서비스 콜백 함수 정의
        if not self.qrdata or not self.package_info:  # QR 데이터 또는 패키지 정보가 없는 경우
            self.get_logger().info("QR data is not detected..")  # QR 데이터 없음 로그 출력
            response.package_id = -2  # 응답에 오류 코드 -2 설정
            return response  # 응답 반환
        
        package = self.package_info.pop(0)  # 패키지 정보 리스트에서 첫 번째 항목 제거 및 반환
        package_id = self.qrdata.pop(0)  # QR 데이터 리스트에서 첫 번째 항목 제거 및 반환
        self.complated_package.append(package_id)  # 완료된 패키지 리스트에 ID 추가

        if package_id != package.packageid:  # QR 데이터와 패키지 ID가 일치하지 않는 경우
            self.get_logger().info("Package ID is not matched..")  # ID 불일치 로그 출력
            response.package_id = -1  # 응답에 오류 코드 -1 설정
            return response  # 응답 반환
        
        self.get_logger().info("received service package info")  # 서비스 요청 수신 로그 출력
        response.package_id = package.packageid  # 응답에 패키지 ID 설정
        response.region = package.region  # 응답에 지역 설정
        response.width = package.width  # 응답에 너비 설정
        response.depth = package.depth  # 응답에 깊이 설정
        response.height = package.height  # 응답에 높이 설정
        response.process = package.process  # 응답에 처리 상태 설정

        #package position get
        center, theta = self.get_package_position(package_id)  # 패키지의 중심 좌표와 각도를 계산
        response.x = center[0]  # 응답에 X 좌표 설정
        response.y = center[1]  # 응답에 Y 좌표 설정
        response.theta = theta  # 응답에 회전 각도 설정
        self.get_logger().info("service response: " + str(response))  # 응답 내용 로그 출력
        return response  # 응답 반환
    
    def get_package_position(self, package_id):  # 패키지의 위치와 각도를 계산하는 메서드 정의
        while True:  # 무한 루프 시작 (패키지 위치를 찾을 때까지 반복)
            with self.cap_lock:  # 카메라 락을 사용하여 스레드 안전성 보장
                ret, frame = self.cap.read()  # 카메라에서 프레임 읽기
            frame = cv2.flip(frame,0)  # 프레임을 수직으로 뒤집음
            frame = cv2.flip(frame,1)  # 프레임을 수평으로 뒤집음
            frame = cv2.undistort(frame, mtx, dist)  # 카메라 왜곡 보정 적용

            # 프레임 중앙 부분만 자르기
            height, width = frame.shape[:2]  # 프레임의 높이와 너비 추출
            # 원하는 중앙 영역의 너비 설정 (예: 전체 너비의 50%)
            center_width = int(width * 0.5)  # 중앙 영역 너비를 전체 너비의 50%로 설정
            # 중앙 좌표 계산
            center_x = width // 2  # 프레임의 X축 중앙 좌표 계산
            # 잘라낼 영역의 시작점과 끝점 계산
            start_x = center_x - (center_width // 2)  # 자를 영역의 시작 X 좌표 계산
            end_x = center_x + (center_width // 2)  # 자를 영역의 끝 X 좌표 계산
            # 프레임 자르기
            frame = frame[:, start_x:end_x]  # 프레임에서 중앙 부분만 자름

            result_center = None  # 결과 중심 좌표를 저장할 변수 초기화
            result_angle = None  # 결과 각도를 저장할 변수 초기화

            decodedObjects = pyzbar.decode(frame)  # 프레임에서 QR 코드 디코딩
            if decodedObjects:  # QR 코드가 감지된 경우
                for obj in decodedObjects:  # 감지된 QR 코드 객체 순회
                    if package_id != int(obj.data.decode('utf-8')):  # 요청된 패키지 ID와 QR 데이터가 다르면 건너뜀
                        continue  # 다음 객체로 이동
                    points = obj.polygon  # QR 코드의 다각형 좌표 추출
                    # 포인트를 numpy 배열로 변환
                    points_array = np.array([point for point in points], dtype=np.float32)  # 다각형 좌표를 NumPy 배열로 변환
                    # 최소 외접 사각형 계산
                    rect = cv2.minAreaRect(points_array)  # 최소 외접 사각형 계산
                    # rect는 ((center_x, center_y), (width, height), angle) 형태로 반환됨
                    center, dimensions, angle = rect  # 중심, 크기, 각도 추출
                    # 각도가 -90°에서 0° 사이일 경우, 너비와 높이를 교환하고 각도를 조정
                    # if angle < -45:  # 주석 처리된 조건문 (필요 시 활성화 가능)
                    #     width, height = dimensions  # 너비와 높이 교환
                    #     angle += 90  # 각도 조정
                    # else:  # 주석 처리된 조건문
                    #     width, height = dimensions  # 너비와 높이 유지

                    result_center = (center[0], center[1])  # 중심 좌표 저장
                    result_angle = round(angle, 2)  # 각도를 소수점 2자리로 반올림하여 저장
                
                if result_center is not None:  # 중심 좌표가 계산된 경우
                    break  # 루프 종료

        return result_center, result_angle  # 중심 좌표와 각도 반환

    def detect_from_camera(self):  # 카메라에서 QR 코드를 감지하는 메서드 정의
        global mtx, dist  # 전역 변수 mtx와 dist 사용 선언
        while self.camera_thread_running:  # 카메라 스레드가 실행 중일 때 반복
            with self.cap_lock:  # 카메라 락을 사용하여 스레드 안전성 보장
                ret, frame = self.cap.read()  # 카메라에서 프레임 읽기
            if not ret:  # 프레임 읽기 실패 시
                self.get_logger().info("Camera error..")  # 카메라 오류 로그 출력
                break  # 루프 종료
            frame = cv2.flip(frame,0)  # 프레임을 수직으로 뒤집음
            frame = cv2.flip(frame,1)  # 프레임을 수평으로 뒤집음
            frame = cv2.undistort(frame, mtx, dist)  # 카메라 왜곡 보정 적용

            # 프레임 중앙 부분만 자르기
            height, width = frame.shape[:2]  # 프레임의 높이와 너비 추출
            # 원하는 중앙 영역의 너비 설정 (예: 전체 너비의 50%)
            center_width = int(width * 0.5)  # 중앙 영역 너비를 전체 너비의 50%로 설정
            # 중앙 좌표 계산
            center_x = width // 2  # 프레임의 X축 중앙 좌표 계산
            # 잘라낼 영역의 시작점과 끝점 계산
            start_x = center_x - (center_width // 2)  # 자를 영역의 시작 X 좌표 계산
            end_x = center_x + (center_width // 2)  # 자를 영역의 끝 X 좌표 계산
            # 프레임 자르기
            frame = frame[:, start_x:end_x]  # 프레임에서 중앙 부분만 자름

            decodedObjects = pyzbar.decode(frame)  # 프레임에서 QR 코드 디코딩
            if decodedObjects:  # QR 코드가 감지된 경우
                for obj in decodedObjects:  # 감지된 QR 코드 객체 순회
                    points = obj.polygon  # QR 코드의 다각형 좌표 추출
                    # 포인트를 numpy 배열로 변환
                    points_array = np.array([point for point in points], dtype=np.float32)  # 다각형 좌표를 NumPy 배열로 변환
                    # 최소 외접 사각형 계산
                    rect = cv2.minAreaRect(points_array)  # 최소 외접 사각형 계산
                    # rect는 ((center_x, center_y), (width, height), angle) 형태로 반환됨
                    center, dimensions, angle = rect  # 중심, 크기, 각도 추출
                    # 각도가 -90°에서 0° 사이일 경우, 너비와 높이를 교환하고 각도를 조정
                    # if angle < -45:  # 주석 처리된 조건문 (필요 시 활성화 가능)
                    #     width, height = dimensions  # 너비와 높이 교환
                    #     angle += 90  # 각도 조정
                    # else:  # 주석 처리된 조건문
                    #     width, height = dimensions  # 너비와 높이 유지

                    center = (int(center[0]), int(center[1]))  # 중심 좌표를 정수로 변환
                    # 최소 외접 사각형 시각화
                    # box = cv2.boxPoints(rect)  # 주석 처리된 코드: 사각형 꼭짓점 계산
                    # box = np.int0(box)  # 주석 처리된 코드: 꼭짓점을 정수로 변환
                    # cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)  # 주석 처리된 코드: 사각형 그리기
                    
                    # 중심점 표시
                    cv2.circle(frame, center, 2, (0, 255, 0), -1)  # 중심점에 초록색 원 그리기
                    
                    # 화면 중심점과 QR 코드 중심점 사이의 거리 계산
                    dx = center[0] - self.forces_x  # X축 거리 계산
                    dy = center[1] - self.forces_y  # Y축 거리 계산
                    cv2.putText(frame, f"({dx}, {dy}, {int(angle)})", (center[0] + 10, center[1] - 10),  # 거리와 각도를 프레임에 텍스트로 표시
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)  # 텍스트 스타일 설정
                    
                    #cv2.putText(frame, f"Center: {center[0], center[1]}", (center[0] + 10, center[1] + 10),  # 주석 처리된 코드: 중심 좌표 표시
                    #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)  # 주석 처리된 텍스트 스타일
                    
                    qrdata = int(obj.data.decode('utf-8'))  # QR 코드 데이터를 정수로 디코딩
                    if qrdata in self.qrdata or qrdata in self.complated_package:  # 이미 처리된 QR 코드면 건너뜀
                        continue  # 다음 객체로 이동
                    with self.qrdata_lock:  # QR 데이터 락을 사용하여 스레드 안전성 보장
                        self.qrdata.append(qrdata)  # 새로운 QR 데이터를 리스트에 추가
                    self.get_logger().info(f'QR data: {qrdata}')  # 감지된 QR 데이터 로그 출력
                    self.socket_client.send_message(f"{qrdata}\n")  # 소켓으로 QR 데이터 전송

            cv2.line(frame, (self.forces_x, 0), (self.forces_x, self.height), (255, 0, 0), 1)  # 프레임에 수직선 그리기 (중앙 표시)
            cv2.line(frame, (0, self.forces_y), (self.width, self.forces_y), (255, 0, 0), 1)  # 프레임에 수평선 그리기 (중앙 표시)

            cv2.imshow('frame', frame)  # 프레임을 화면에 표시
            cv2.waitKey(1)  # 키 입력 대기 (1ms)
            
            time.sleep(0.03)  # 0.03초 대기 (프레임 속도 조절)
    
    def destroy_node(self):  # 노드 종료 메서드 정의
        self.socket_client.running = False  # 소켓 클라이언트 실행 플래그 비활성화
        if self.socket_client.receive_thread.is_alive():  # 소켓 수신 스레드가 실행 중이면
            self.socket_client.receive_thread.join()  # 수신 스레드 종료 대기

        self.camera_thread_running = False  # 카메라 스레드 실행 플래그 비활성화
        if self.camera_thread.is_alive():  # 카메라 스레드가 실행 중이면
            self.camera_thread.join()  # 카메라 스레드 종료 대기
        super().destroy_node()  # 부모 클래스(Node)의 종료 메서드 호출
        self.cap.release()  # 카메라 리소스 해제
        cv2.destroyAllWindows()  # 모든 OpenCV 창 닫기

class SoketClient():  # 소켓 클라이언트 클래스 정의
    def __init__(self, qr_node):  # 생성자 정의, QR 노드 객체를 인자로 받음
        # 소켓 초기화
        self.client = None  # 소켓 객체 초기화
        self.running = True  # 소켓 실행 상태 플래그 초기화
        self.qr_node = qr_node  # QR 노드 객체 저장

        self.ip_address = SERVER_IP  # 서버 IP 주소 설정
        self.port = SERVER_PORT  # 서버 포트 번호 설정

        self.connect_to_server()  # 서버에 연결 시도

        # 수신 스레드 시작
        self.receive_thread = threading.Thread(target=self.receive_messages)  # 수신 스레드 생성
        self.receive_thread.daemon = True  # 수신 스레드를 데몬 모드로 설정
        self.receive_thread.start()  # 수신 스레드 시작

    def connect_to_server(self):  # 서버 연결 메서드 정의
        """서버에 연결"""
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP 소켓 객체 생성
        try:  # 연결 시도
            self.client.connect((self.ip_address, self.port))  # 지정된 IP와 포트로 서버 연결
            self.qr_node.get_logger().info(f"{self.ip_address} Connect Server..")  # 연결 성공 로그 출력
        except ConnectionRefusedError:  # 연결 실패 시 예외 처리
            #서버 연결 실패 로그
            self.qr_node.get_logger().info(f"{self.ip_address} Connect Fail..")  # 연결 실패 로그 출력
            self.running = False  # 실행 플래그 비활성화

    def receive_messages(self):  # 서버로부터 메시지를 수신하는 메서드 정의
        while self.running:  # 소켓이 실행 중일 때 반복
            try:  # 메시지 수신 시도
                response = self.client.recv(1024).decode('utf-8')  # 서버로부터 최대 1024바이트 수신 및 디코딩
                if not response:  # 수신 데이터가 없으면 (연결 끊김)
                    self.qr_node.get_logger().info("disconnect server")  # 서버 연결 끊김 로그 출력
                    break  # 루프 종료

                # 🔹 데이터 파싱
                package = self.parse_server_data(response)  # 수신 데이터를 파싱하여 패키지 객체 생성
                if package:  # 유효한 패키지 객체가 생성된 경우
                    self.qr_node.get_logger().info(f"Package: {package}")  # 패키지 정보 로그 출력
                    with self.qr_node.package_info_lock:  # 패키지 정보 락을 사용하여 스레드 안전성 보장
                        self.qr_node.package_info.append(package)  # 패키지 정보를 리스트에 추가

            except ConnectionResetError:  # 연결이 강제로 끊긴 경우
                self.qr_node.get_logger().info("disconnect server")  # 서버 연결 끊김 로그 출력
                break  # 루프 종료

    def send_message(self, message):  # 서버에 메시지를 전송하는 메서드 정의
        """서버에 메시지 전송"""
        try:  # 메시지 전송 시도
            self.client.sendall(message.encode('utf-8'))  # 메시지를 UTF-8로 인코딩하여 전송
        except Exception as e:  # 전송 중 오류 발생 시
            self.qr_node.get_logger().info(f"send message error: {e}")  # 오류 로그 출력

    def parse_server_data(self, response):  # 서버 응답을 파싱하는 메서드 정의
        """Parses the server response into a `Packageinfo` object"""
        try:  # 파싱 시도
            parts = response.split(", ")  # 응답을 쉼표와 공백으로 분리
            data = {}  # 파싱된 데이터를 저장할 딕셔너리 초기화

            # Parse each "key: value" pair
            for part in parts:  # 각 부분 순회
                key_value = part.split(": ")  # 키와 값을 콜론과 공백으로 분리
                if len(key_value) == 2:  # 키-값 쌍이 유효한 경우
                    key, value = key_value  # 키와 값 분리
                    data[key.strip()] = value.strip()  # 공백 제거 후 딕셔너리에 저장

            # Convert extracted values correctly
            return Packageinfo(  # 파싱된 데이터를 기반으로 Packageinfo 객체 생성
                packageid=int(data.get("packageID", "")),  # 패키지 ID를 정수로 변환
                region=data.get("Region", ""),  # 지역 문자열 추출
                width=float(data.get("width", 0.0)),  # 너비를 실수로 변환
                depth=float(data.get("depth", 0.0)),  # 깊이를 실수로 변환
                height=float(data.get("height", 0.0)),  # 높이를 실수로 변환
                process=int(data.get("process", ""))  # 처리 상태를 정수로 변환
            )

        except ValueError as e:  # 데이터 변환 오류 발생 시
            print(f"Data conversion error: {e}")  # 오류 메시지 출력
            return None  # None 반환

        except Exception as e:  # 기타 파싱 오류 발생 시
            print(f"Parsing error: {e}")  # 오류 메시지 출력
            return None  # None 반환

def main(args=None):  # 메인 함수 정의
    rclpy.init(args=args)  # ROS 2 환경 초기화
    qr_node = Detect_Send()  # Detect_Send 노드 객체 생성
    try:  # 노드 실행 시도
        rclpy.spin(qr_node)  # 노드 실행 (이벤트 루프 시작)
    except KeyboardInterrupt:  # 키보드 인터럽트(Ctrl+C) 발생 시
        qr_node.get_logger().info("KeyboardInterrupt Stop.")  # 종료 로그 출력
    finally:  # 종료 시 항상 실행
        qr_node.destroy_node()  # 노드 종료 및 리소스 해제
        rclpy.shutdown()  # ROS 2 환경 종료

if __name__ == "__main__":  # 스크립트가 직접 실행될 때
    main()  # 메인 함수 호출
