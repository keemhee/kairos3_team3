import rclpy  # ROS 2의 Python 클라이언트 라이브러리를 가져옴
from rclpy.node import Node  # ROS 2 노드 클래스를 가져옴
import cv2  # OpenCV 라이브러리를 가져옴 (이미지 처리용)
import time  # 시간 관련 기능을 사용하기 위해 가져옴
import threading  # 멀티스레딩을 위해 가져옴
import numpy as np  # 수학적 계산 및 배열 처리를 위한 NumPy 라이브러리를 가져옴
from pymycobot.myagv import MyAgv  # MyCobot AGV 제어 라이브러리를 가져옴
from enum import Enum  # 열거형(Enum) 클래스를 가져옴
import math  # 수학 연산을 위한 모듈을 가져옴
from std_msgs.msg import Bool  # ROS 2 표준 메시지 타입 Bool을 가져옴

prev_direction = None  # 이전 방향을 저장하는 전역 변수 초기화
direction = None  # 현재 방향을 저장하는 전역 변수 초기화
direction_lock = threading.Lock()  # 방향 데이터 접근을 위한 락 객체 생성 (스레드 안전성 보장)

mtx = np.loadtxt('src/line_tracing_agv/line_tracing_agv/camera_calibration/mtx.csv', delimiter=',')  # 카메라 캘리브레이션 행렬을 CSV 파일에서 불러옴
dist = np.loadtxt('src/line_tracing_agv/line_tracing_agv/camera_calibration/dist.csv', delimiter=',')  # 카메라 왜곡 계수를 CSV 파일에서 불러옴
error = 0.0  # 위치 오차를 저장하는 전역 변수 초기화
pid_output = 0  # PID 제어 출력 값을 저장하는 전역 변수 초기화
prev_error = 0  # 이전 오차를 저장하는 전역 변수 초기화
integral = 0  # 적분 값을 저장하는 전역 변수 초기화

diff_ang = 0.0  # 각도 차이를 저장하는 전역 변수 초기화
angle_prev_error = 0  # 이전 각도 오차를 저장하는 전역 변수 초기화
angle_integral = 0  # 각도 적분 값을 저장하는 전역 변수 초기화

base_speed = 128  # AGV의 기본 속도를 상수로 정의

Kp = 0.2  # PID 비례 게인 설정 (반응 속도 조정)
Ki = 0.01  # PID 적분 게인 설정 (누적 오차 조정)
Kd = 0.4  # PID 미분 게인 설정 (변화율 민감도 조정)

# Yellow color range
lower_yellow = np.array([20, 70, 100], dtype=np.uint8)  # 노란색 하한 값 (HSV 색상 공간) 정의
upper_yellow = np.array([70, 255, 255], dtype=np.uint8)  # 노란색 상한 값 (HSV 색상 공간) 정의

# 빨간색 범위 1 (낮은 Hue 값)
lower_red1 = np.array([0, 50, 50], dtype=np.uint8)  # 빨간색 하한 값 1 (HSV 색상 공간) 정의
upper_red1 = np.array([10, 255, 255], dtype=np.uint8)  # 빨간색 상한 값 1 (HSV 색상 공간) 정의

# 빨간색 범위 2 (높은 Hue 값)
lower_red2 = np.array([170, 50, 50], dtype=np.uint8)  # 빨간색 하한 값 2 (HSV 색상 공간) 정의
upper_red2 = np.array([180, 255, 255], dtype=np.uint8)  # 빨간색 상한 값 2 (HSV 색상 공간) 정의

class AgvStep(Enum):  # AGV 상태를 정의하는 열거형 클래스 정의
    Driving = 1  # 주행 중 상태
    Parking_Step1 = 2  # 주차 단계 1
    Parking_Step1_End = 3  # 주차 단계 1 종료
    Parking_Step2 = 4  # 주차 단계 2
    Stop = 5  # 정지 상태

class AgvNode(Node):  # ROS 2 노드를 상속받아 AGV 제어 클래스 정의
    def __init__(self):  # 노드 초기화 메서드 정의
        super().__init__("Agv_Node")  # 부모 클래스(Node)의 생성자를 호출하며 노드 이름 설정
        self.get_logger().info("Agv_Node has been started..")  # 노드 시작 로그 출력
        self.is_agv_ready = False  # AGV 준비 상태 플래그 초기화

        self.agv_ready_publisher = self.create_publisher(Bool, "agv_ready", 10)  # AGV 준비 상태 퍼블리셔 생성
        self.plc_complate_subscription = self.create_subscription(Bool, "plc_complate", self.plc_complate_callback, 10)  # PLC 완료 토픽 구독 설정

        self.cap = cv2.VideoCapture(0)  # 기본 카메라(인덱스 0)를 열어 비디오 캡처 객체 생성
        self.agv = MyAgv("/dev/ttyAMA2", 115200)  # MyAgv 객체 생성 (시리얼 포트와 통신 속도 설정)

        self._running = True  # 노드 실행 상태 플래그 초기화
        self.agvStep = AgvStep.Driving  # AGV 초기 상태를 주행 중으로 설정
        self.agv.set_led(2, 0, 0, 255)  # AGV LED를 파란색으로 설정 (상태 표시)

        self.camera_thread = threading.Thread(target=self.camera_thread, daemon=True)  # 카메라 처리 스레드 생성
        self.agv_controll_thread = threading.Thread(target=self.agv_controll_thread, daemon=True)  # AGV 제어 스레드 생성

        self.camera_thread.start()  # 카메라 스레드 시작
        time.sleep(5)  # 5초 대기 (카메라 초기화 안정화)
        self.agv_controll_thread.start()  # AGV 제어 스레드 시작
    
    def calculate_pid(self, error, prev_error, integral, prev_output=0):  # PID 제어 값을 계산하는 메서드 정의
        global Kp, Ki, Kd  # 전역 PID 게인 변수 사용 선언

        derivative = error - prev_error  # 미분 값 계산 (오차 변화율)
        output = Kp * error + Ki * integral + Kd * derivative  # PID 출력 계산
        output = max(min(int(output), 127), -127)  # 출력 값을 -127에서 127 사이로 제한
        smoothed_output = 0.7 * prev_output + 0.3 * output  # 이전 출력과 현재 출력을 혼합하여 스무딩
        return int(smoothed_output)  # 정수형으로 변환된 출력 반환
    
    def calculate_turn_speed(self, pid_output):  # 회전 속도를 계산하는 메서드 정의
        min_turn_speed = 20  # 최소 회전 속도 정의
        max_turn_speed = 100  # 최대 회전 속도 정의
        scale_factor = 1.1  # 회전 속도 증폭 비율 정의
        # PID 출력값에 따라 선형 비례 계산 (증폭 적용)
        turn_speed = min_turn_speed + (max_turn_speed - min_turn_speed) * abs(pid_output) / 127  # 회전 속도 계산
        return int(turn_speed * scale_factor)  # 증폭된 정수형 회전 속도 반환

    def process_frame(self, roi):  # 프레임을 처리하여 AGV 방향을 결정하는 메서드 정의
        global direction, prev_direction, pid_output, lower_yellow, upper_yellow, lower_red1, upper_red1, prev_error, integral, upper_red2, upper_red2, error, diff_ang, angle_prev_error, angle_integral, direction_lock  # 전역 변수 사용 선언
        # height, _, _= frame.shape  # 주석 처리된 코드: 프레임 높이 추출

        # roi_height = int(height / 6)  # 주석 처리된 코드: ROI 높이 계산
        # roi_top = height - roi_height  # 주석 처리된 코드: ROI 상단 위치 계산
        # roi = frame[roi_top:, :]  # 주석 처리된 코드: ROI 추출
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)  # ROI를 HSV 색상 공간으로 변환

        # 모폴로지 연산에 사용할 커널 정의
        kernel = np.ones((5, 5), np.uint8)  # 5x5 크기의 모폴로지 연산 커널 생성

        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  # 노란색 마스크 생성
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)  # 열림 연산으로 노이즈 제거
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)  # 닫힘 연산으로 구멍 메우기

        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)  # 빨간색 마스크 1 생성
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)  # 빨간색 마스크 2 생성

        red_mask = red_mask1 | red_mask2  # 두 빨간색 마스크를 결합
        
        yellow_detect = cv2.countNonZero(yellow_mask) > 100  # 노란색 영역 감지 (픽셀 수 기준)
        red_detect = cv2.countNonZero(red_mask) > 100  # 빨간색 영역 감지 (픽셀 수 기준)

        roi_h, roi_w = roi.shape[:2]  # ROI의 높이와 너비 추출
        # ROI의 중심 세로선을 빨간색으로 그립니다.
        cv2.line(roi, (roi_w // 2, 0), (roi_w // 2, roi_h), (250, 0, 0), 2)  # ROI 중심에 빨간색 수직선 그리기
        
        if red_detect:  # 빨간색이 감지된 경우
            #정지선 만남 Dirving상태였다면 Parking_Step1 돌입
            if self.agvStep == AgvStep.Driving:  # 현재 상태가 주행 중이면
                self.agvStep = AgvStep.Parking_Step1  # 주차 단계 1로 전환
                self.agv.set_led(2, 255, 0, 0)  # LED를 빨간색으로 설정 (상태 표시)
                parking_step1_thread = threading.Thread(target=self.parking_step1, daemon=True)  # 주차 단계 1 스레드 생성
                parking_step1_thread.start()  # 주차 단계 1 스레드 시작
        elif yellow_detect:  # 노란색이 감지된 경우
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)  # ROI를 그레이스케일로 변환
            _, binary_image = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)  # 이진화 이미지 생성
            yellow_binary_image = cv2.bitwise_and(binary_image, binary_image, mask=yellow_mask)  # 노란색 영역만 추출
            yellow_contours, _ = cv2.findContours(yellow_binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 노란색 윤곽선 추출
            if len(yellow_contours) >= 1:  # 윤곽선이 하나 이상 존재하면
                max_contour = max(yellow_contours, key=cv2.contourArea)  # 가장 큰 윤곽선 선택
                M = cv2.moments(max_contour)  # 윤곽선의 모멘트 계산

                if M["m00"] != 0:  # 모멘트 값이 0이 아니면 (유효한 윤곽선)
                    cx = int(M["m10"] / M["m00"])  # 중심 X 좌표 계산
                    #cy = int(M["m01"] / M["m00"])  # 주석 처리된 코드: 중심 Y 좌표 계산

                    center_points = []  # 중심점 리스트 초기화
                    for y in range(roi_h):  # ROI 높이만큼 반복
                        # 해당 행에서 흰색 픽셀(노란색 영역) 인덱스 찾기
                        xs = np.where(yellow_mask[y, :] > 0)[0]  # 해당 행의 노란색 픽셀 위치 추출
                        if len(xs) > 0:  # 노란색 픽셀이 존재하면
                            # 흰색 픽셀의 평균 x 좌표 계산
                            x_avg = int(np.mean(xs))  # 평균 X 좌표 계산
                            center_points.append((x_avg, y))  # 중심점 리스트에 추가

                    # 충분한 점이 있다면, 이 점들을 선으로 이어서 그리기
                    if len(center_points) > 1:  # 중심점이 2개 이상이면
                        pts = np.array(center_points, np.int32)  # 중심점을 NumPy 배열로 변환
                        pts = pts.reshape((-1, 1, 2))  # OpenCV에서 사용할 형태로 재구성

                        #두 선의 각도
                        line = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)  # 직선 피팅
                        vx, vy, x0, y0 = line.flatten()  # 직선의 방향 벡터와 기준점 추출

                        # ROI 높이 roi_h 가정, y=0과 y=roi_h에서의 두 점을 계산
                        t0 = (0 - y0) / vy if vy != 0 else 0  # 상단 점의 t 값 계산
                        t1 = (roi_h - y0) / vy if vy != 0 else 0  # 하단 점의 t 값 계산

                        x_top = x0 + vx * t0  # 상단 X 좌표 계산
                        x_bottom = x0 + vx * t1  # 하단 X 좌표 계산

                        # 계산된 두 점을 사용하여 직선을 그림 (여기서는 파란색 선, 두께 2)
                        cv2.line(roi, (int(x_top), 0), (int(x_bottom), roi_h), (255, 0, 0), 2)  # 직선 그리기

                        # 이미 x_top, x_bottom, roi_h가 계산되어 있음
                        dx = x_bottom - x_top  # X축 차이 계산
                        dy = roi_h  # Y축 차이 (ROI 높이)
                        blue_line_angle = math.degrees(math.atan2(dx, dy))  # 수직 기준 상대 각도 계산

                    frame_center = yellow_binary_image.shape[1] // 2  # 프레임 중심 X 좌표 계산
                    error = cx - frame_center  # 중심 오차 계산

                    with direction_lock:  # 방향 락을 사용하여 스레드 안전성 보장
                        #diff_ang = blue_line_angle  # 주석 처리된 코드: 각도 차이 직접 설정
                        pid_output = self.calculate_pid(error, prev_error, integral)  # 위치 PID 출력 계산
                        prev_error = error  # 이전 오차 업데이트
                        integral += error  # 적분 값 누적
                        integral = max(min(integral, 100), -100)  # 적분 값을 -100에서 100 사이로 제한

                        diff_ang = self.calculate_pid(blue_line_angle, angle_prev_error, angle_integral)  # 각도 PID 출력 계산
                        angle_prev_error = blue_line_angle  # 이전 각도 오차 업데이트
                        angle_integral += blue_line_angle  # 각도 적분 값 누적
                        angle_integral = max(min(angle_integral, 100), -100)  # 각도 적분 값을 -100에서 100 사이로 제한

                        cv2.putText(roi, f"pid_output : {pid_output}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)  # PID 출력값 표시
                        cv2.putText(roi, f"diff_ang : {diff_ang}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)  # 각도 차이 표시

                        if self.agvStep == AgvStep.Driving:  # 주행 중 상태일 때
                            if pid_output > 3: # Turn right  # PID 출력이 3 초과면 우회전
                                if prev_direction != "RIGHT":  # 이전 방향이 우회전이 아니면
                                    self.agv.stop()  # AGV 정지
                                direction = "RIGHT"  # 방향을 우회전으로 설정
                            elif pid_output < -3:  # Turn left  # PID 출력이 -3 미만이면 좌회전
                                if prev_direction != "LEFT":  # 이전 방향이 좌회전이 아니면
                                    self.agv.stop()  # AGV 정지
                                direction = "LEFT"  # 방향을 좌회전으로 설정
                            else:  # 직진 조건
                                if prev_direction != "FOWARD":  # 이전 방향이 직진이 아니면
                                    self.agv.stop()  # AGV 정지 
                                direction = "FOWARD"  # 방향을 직진으로 설정
                    
                    if self.agvStep == AgvStep.Parking_Step1_End:  # 주차 단계 1 종료 시
                        self.agvStep = AgvStep.Parking_Step2  # 주차 단계 2로 전환
                        parking_step2_thread = threading.Thread(target=self.parking_step2, daemon=True)  # 주차 단계 2 스레드 생성
                        parking_step2_thread.start()  # 주차 단계 2 스레드 시작
        else:  # 노란색과 빨간색 모두 감지되지 않은 경우
            direction = None  # 방향을 None으로 설정
            if self.agvStep == AgvStep.Parking_Step2:  # 주차 단계 2일 때
                with direction_lock:  # 방향 락을 사용하여 스레드 안전성 보장
                    pid_output = -100  # PID 출력을 강제로 -100으로 설정
                    diff_ang = -100  # 각도 차이를 강제로 -100으로 설정


        cv2.imshow("roi", roi)  # ROI 창 표시
        cv2.waitKey(1)  # 키 입력 대기 (1ms, GUI 이벤트 처리)

    def camera_thread(self):  # 카메라 데이터를 처리하는 스레드 메서드 정의
        global mtx, dist  # 전역 변수 mtx와 dist 사용 선언
        while self._running:  # 노드가 실행 중일 때 반복                                                                
            ret, frame = self.cap.read()  # 카메라에서 프레임 읽기

            if not ret:  # 프레임 읽기 실패 시
                self.get_logger().info("Camera error..")  # 카메라 오류 로그 출력
                break  # 루프 종료
            frame = cv2.undistort(frame, mtx, dist)  # 카메라 왜곡 보정 적용
            height, width, _ = frame.shape  # 프레임의 높이와 너비 추출

            # 관심 영역(ROI) 추출 (예: 하단 1/6 영역)
            roi_height = int(height / 6)  # ROI 높이를 프레임 높이의 1/6으로 설정
            roi_top = height - roi_height  # ROI 상단 위치 계산
            roi = frame[roi_top:, :]  # ROI 추출
            roi_h, roi_w = roi.shape[:2]  # ROI의 높이와 너비 추출

            src_pts = np.float32([  # 원본 ROI의 네 점 정의 (사다리꼴 형태)
                [int(roi_w * 0.25), 0],      # 좌측 상단 (너비의 25%)
                [int(roi_w * 0.75), 0],      # 우측 상단 (너비의 75%)
                [roi_w, roi_h],              # 우측 하단
                [0, roi_h]                   # 좌측 하단
            ])

            # dst_pts: 보정 후 정면 뷰의 네 점 (직사각형)
            dst_pts = np.float32([  # 변환 후 직사각형 네 점 정의
                [0, 0],  # 좌측 상단
                [roi_w, 0],  # 우측 상단
                [roi_w, roi_h],  # 우측 하단
                [0, roi_h]  # 좌측 하단
            ])

            H_matrix, _ = cv2.findHomography(src_pts, dst_pts)  # 호모그래피 행렬 계산
            corrected_roi = cv2.warpPerspective(roi, H_matrix, (roi_w, roi_h))  # 원근 변환으로 ROI 보정

            self.process_frame(corrected_roi)  # 보정된 ROI 처리
            time.sleep(0.03)  # 0.03초 대기 (프레임 속도 조절)

    def parking_step1(self):  # 주차 단계 1 메서드 정의
        global base_speed  # 전역 변수 base_speed 사용 선언
        #마지막 까지 왔다 뒤로 돌아서 주차 하자
        self.agv.stop()  # AGV 정지
        time.sleep(1)  # 1초 대기
        self.agv.go_ahead(1, timeout=2.0)  # AGV를 2초 동안 전진
        time.sleep(1)  # 1초 대기
        for i in range(50):  # 50번 반복 (약 5초 동안 회전)
            self.agv._mesg(base_speed, base_speed, base_speed+10)  # AGV를 약간 오른쪽으로 회전
            time.sleep(0.1)  # 0.1초 대기
        
        self.agv.stop()  # AGV 정지
        self.agvStep = AgvStep.Parking_Step1_End  # 주차 단계 1 종료 상태로 전환
    
    def parking_step2(self):  # 주차 단계 2 메서드 정의
        global base_speed, diff_ang, pid_output, direction_lock  # 전역 변수 사용 선언
        threshold_distance = 2  # 중심 오차 허용 범위 (픽셀 단위)
        threshold_angle = 1  # 각도 오차 허용 범위 (도 단위)
        max_attempts = 10  # 최대 조정 시도 횟수
        angle_attempt = 0  # 각도 조정 시도 횟수 초기화
        distance_attempt = 0  # 거리 조정 시도 횟수 초기화

        self.get_logger().info("[INFO] Aligning AGV to center of the yellow area...")  # 정렬 시작 로그 출력
        time.sleep(5)  # 5초 대기 (안정화)

        while angle_attempt < max_attempts:  # 최대 시도 횟수까지 각도 조정 반복
            with direction_lock:  # 방향 락을 사용하여 스레드 안전성 보장
                #local_distance = pid_output    # 주석 처리된 코드: 거리 오차
                local_diff_ang = int(diff_ang)  # 각도 차이를 정수로 변환

            if local_diff_ang == -100:  # 각도 데이터가 유효하지 않은 경우
                self.agv.clockwise_rotation(1, timeout=0.5)  # 시계 방향으로 0.5초 회전
                time.sleep(5)  # 5초 대기
                continue  # 다음 반복으로 이동

            self.get_logger().info(f"[DEBUG] Attempt {angle_attempt+1}: Angle error: {local_diff_ang}")  # 디버그 로그 출력

            # 만약 좌우 오차와 각도 오차 모두 허용범위 내라면 정렬 완료
            if abs(local_diff_ang) <= threshold_angle:  # 각도 오차가 허용 범위 내이면
                self.get_logger().info("[INFO] Angle Center Complate!")  # 각도 정렬 완료 로그 출력
                break  # 루프 종료

            # 1. 각도 보정: AGV의 진행 방향(수평)과 노란색 영역 중심선의 각도가 많이 차이날 경우
            if abs(local_diff_ang) > threshold_angle:  # 각도 오차가 허용 범위를 초과하면
                if local_diff_ang > 0:  # 각도가 양수이면 (오른쪽으로 기울어짐)
                    # 예: 노란색 중심선이 오른쪽으로 기울어져 있으면, AGV가 좌측으로 회전하여 보정
                    self.get_logger().info("[INFO] Rotating AGV counterclockwise for alignment")  # 반시계 방향 회전 로그
                    self.agv.counterclockwise_rotation(1, timeout=0.2)  # 0.2초 반시계 방향 회전
                elif local_diff_ang < 0:  # 각도가 음수이면 (왼쪽으로 기울어짐)
                    self.get_logger().info("[INFO] Rotating AGV clockwise for alignment")  # 시계 방향 회전 로그
                    self.agv.clockwise_rotation(1, timeout=0.2)  # 0.2초 시계 방향 회전
            angle_attempt += 1  # 시도 횟수 증가
            time.sleep(5)  # 5초 대기 (조정 후 안정화)

        while distance_attempt < max_attempts:  # 최대 시도 횟수까지 거리 조정 반복
            with direction_lock:  # 방향 락을 사용하여 스레드 안전성 보장
                local_distance = pid_output  # 거리 오차 (PID 출력값)
                #local_diff_ang = int(diff_ang)    # 주석 처리된 코드: 각도 차이

            self.get_logger().info(f"[DEBUG] Attempt {distance_attempt+1}: distance error: {local_distance}")  # 디버그 로그 출력

            if abs(local_distance) <= threshold_distance:  # 거리 오차가 허용 범위 내이면
                self.get_logger().info("[INFO] Distance Center Complate!")  # 거리 정렬 완료 로그 출력
                break  # 루프 종료

            # 2. 좌우 위치 보정: 노란색 중심과의 수평 오차가 클 경우
            if abs(local_distance) > threshold_distance:  # 거리 오차가 허용 범위를 초과하면
                if local_distance > 0:  # 노란색 중심이 왼쪽에 있으면
                    # AGV 기준선보다 노란색 영역이 왼쪽에 있다면, AGV를 오른쪽으로 이동시켜야 함
                    self.get_logger().info("[INFO] Adjusting lateral position: Moving RIGHT")  # 오른쪽 이동 로그
                    self.agv.pan_right(1, timeout=0.3)  # 0.3초 오른쪽 이동
                elif local_distance < 0:  # 노란색 중심이 오른쪽에 있으면
                    self.get_logger().info("[INFO] Adjusting lateral position: Moving LEFT")  # 왼쪽 이동 로그
                    self.agv.pan_left(1, timeout=0.3)  # 0.3초 왼쪽 이동

                distance_attempt += 1  # 시도 횟수 증가
                time.sleep(5)  # 5초 대기 (조정 후 안정화)

        #self.get_logger().info("[WARN] Maximum alignment attempts reached. AGV may not be perfectly centered.")  # 주석 처리된 코드: 정렬 실패 경고
        self.agv.stop()  # AGV 정지
        self.is_agv_ready = True  # AGV 준비 상태로 설정

        #주차 완료 pub
        self.pub_agv_ready()  # AGV 준비 상태 퍼블리시
        self.agv.set_led(1, 255, 0, 0)  # LED를 빨간색으로 설정 (완료 표시)

    def pub_agv_ready(self):  # AGV 준비 상태를 퍼블리시하는 메서드 정의
        msg = Bool()  # Bool 메시지 객체 생성
        msg.data = self.is_agv_ready  # 준비 상태 설정
        self.get_logger().info(f'pub: {msg.data}')  # 퍼블리시 데이터 로그 출력

        self.agv_ready_publisher.publish(msg)  # 메시지 퍼블리시

    def plc_complate_callback(self, msg):  # PLC 완료 토픽 콜백 함수 정의
        self.get_logger().info(f'sub: {msg.data}')  # 수신 데이터 로그 출력
        data = msg.data  # 수신 데이터 추출
        
        if data is True and self.is_agv_ready is True:  # PLC 완료 신호와 AGV 준비 상태가 모두 True이면
            #PLC sub로 부터 완료 topic 받으면 해줘야 할 것들들
            self.is_agv_ready = False  # AGV 준비 상태 비활성화
            self.agvStep = AgvStep.Driving  # AGV 상태를 주행 중으로 전환
            self.agv.set_led(2, 0, 0, 255)  # LED를 파란색으로 설정 (주행 상태 표시)

    def agv_controll_thread(self):  # AGV 제어를 처리하는 스레드 메서드 정의
        global pid_output, direction, prev_direction, error, base_speed  # 전역 변수 사용 선언
        no_detect_count = 0  # 라인 미감지 횟수 초기화
        max_no_detect_count = 20  # 최대 미감지 허용 횟수 정의

        while self._running:  # 노드가 실행 중일 때 반복
            if self.agvStep != AgvStep.Driving:  # 주행 중 상태가 아니면
                continue  # 다음 반복으로 이동
            with direction_lock:  # 방향 락을 사용하여 스레드 안전성 보장
                local_pid_output = pid_output  # 로컬 PID 출력값 복사
                local_direction = direction  # 로컬 방향 복사
            turn_speed = int(self.calculate_turn_speed(local_pid_output))  # 회전 속도 계산
            if local_pid_output < -8 and local_pid_output > 8:  # PID 출력이 -8과 8 사이가 아니면
                add_speed = turn_speed  # 추가 속도를 회전 속도로 설정
            elif local_pid_output < -6 and local_pid_output > 6:  # PID 출력이 -6과 6 사이가 아니면
                add_speed = turn_speed // 2  # 추가 속도를 회전 속도의 절반으로 설정
            else:  # 그 외의 경우
                add_speed = turn_speed // 3  # 추가 속도를 회전 속도의 1/3로 설정

            if local_direction == "RIGHT":  # 우회전 방향이면
                #print(f"Turning RIGHT with PID output: {pid_output}")  # 주석 처리된 코드: 디버그 출력
                self.agv._mesg(base_speed + add_speed, base_speed, base_speed - turn_speed)  # AGV 우회전 명령
                prev_direction = "RIGHT"  # 이전 방향을 우회전으로 설정
                no_detect_count = 0  # 미감지 횟수 초기화

            elif local_direction == "LEFT":  # 좌회전 방향이면
                #print(f"Turning LEFT with PID output: {pid_output}")  # 주석 처리된 코드: 디버그 출력
                self.agv._mesg(base_speed + add_speed, base_speed, base_speed + turn_speed)  # AGV 좌회전 명령
                prev_direction = "LEFT"  # 이전 방향을 좌회전으로 설정
                no_detect_count = 0  # 미감지 횟수 초기화
                
            elif local_direction == "FOWARD":  # 직진 방향이면
                #print("Moving FORWARD")  # 주석 처리된 코드: 디버그 출력
                self.agv._mesg(base_speed + 10, base_speed, base_speed)  # AGV 직진 명령
                prev_direction = "FOWARD"  # 이전 방향을 직진으로 설정
                no_detect_count = 0  # 미감지 횟수 초기화
            else:  # 방향이 정의되지 않은 경우
                if prev_direction == "LEFT":  # 이전 방향이 좌회전이면
                    self.agv._mesg(base_speed, base_speed, base_speed + 10)  # 약간 좌회전 유지
                    no_detect_count += 1  # 미감지 횟수 증가
                    if no_detect_count >= max_no_detect_count:  # 최대 미감지 횟수 도달 시
                        self.agv.stop()  # AGV 정지
                        for i in range(max_no_detect_count):  # 우회전으로 조정
                            self.agv._mesg(base_speed, base_speed, base_speed - 10)  # 약간 우회전
                            time.sleep(0.1)  # 0.1초 대기
                        self.agv.stop()  # AGV 정지
                        no_detect_count = 0  # 미감지 횟수 초기화
                        prev_direction = None  # 이전 방향 초기화
                elif prev_direction == "RIGHT":  # 이전 방향이 우회전이면
                    self.agv._mesg(base_speed, base_speed, base_speed - 10)  # 약간 우회전 유지
                    no_detect_count += 1  # 미감지 횟수 증가
                    if no_detect_count >= max_no_detect_count:  # 최대 미감지 횟수 도달 시
                        self.agv.stop()  # AGV 정지
                        for i in range(max_no_detect_count):  # 좌회전으로 조정
                            self.agv._mesg(base_speed, base_speed, base_speed + 10)  # 약간 좌회전
                            time.sleep(0.1)  # 0.1초 대기
                        self.agv.stop()  # AGV 정지
                        prev_direction = None  # 이전 방향 초기화
                        no_detect_count = 0  # 미감지 횟수 초기화
                elif direction is None:  # 방향이 None이면
                    self.agv.stop()  # AGV 정지
                    prev_direction = None  # 이전 방향 초기화
            time.sleep(0.1)  # 0.1초 대기 (제어 주기 조절)

    def destory_node(self):  # 노드 종료 메서드 정의 (오타: destroy_node로 수정 필요)
        self._running = False  # 노드 실행 플래그 비활성화
        if self.camera_thread.is_alive():  # 카메라 스레드가 실행 중이면
            self.camera_thread.join()  # 카메라 스레드 종료 대기
        if self.agv_controll_thread.is_alive():  # AGV 제어 스레드가 실행 중이면
            self.agv_controll_thread.join()  # AGV 제어 스레드 종료 대기

        super().destroy_node()  # 부모 클래스(Node)의 종료 메서드 호출
        self.agv.stop()  # AGV 정지
        self.cap.release()  # 카메라 리소스 해제
        cv2.destroyAllWindows()  # 모든 OpenCV 창 닫기

def main(args=None):  # 메인 함수 정의
    rclpy.init(args=args)  # ROS 2 환경 초기화
    agv_node = AgvNode()  # AgvNode 객체 생성

    try:  # 노드 실행 시도
        rclpy.spin(agv_node)  # 노드 실행 (이벤트 루프 시작)
    except KeyboardInterrupt:  # 키보드 인터럽트(Ctrl+C) 발생 시
        agv_node.get_logger().info("KeyboardInterrupt Stop.")  # 종료 로그 출력
    finally:  # 종료 시 항상 실행
        agv_node.destroy_node()  # 노드 종료 및 리소스 해제
        rclpy.shutdown()  # ROS 2 환경 종료
    
if __name__ == "__main__":  # 스크립트가 직접 실행될 때
    main()  # 메인 함수 호출
