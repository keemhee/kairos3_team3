import rclpy
from rclpy.node import Node
import cv2
import time
import threading
import numpy as np
from pymycobot.myagv import MyAgv
from enum import Enum
import math
from std_msgs.msg import Bool

prev_direction = None
direction = None
direction_lock = threading.Lock()

mtx = np.loadtxt('src/line_tracing_agv/line_tracing_agv/camera_calibration/mtx.csv', delimiter=',')
dist = np.loadtxt('src/line_tracing_agv/line_tracing_agv/camera_calibration/dist.csv', delimiter=',')
error = 0.0
pid_output = 0
prev_error = 0
integral = 0

diff_ang = 0.0
angle_prev_error = 0
angle_integral = 0

base_speed = 128

Kp = 0.2  # 반응 속도를 줄임
Ki = 0.01  # 누적 오차 축소
Kd = 0.4#0.4 0.25  # 변화율에 더 민감

# Yellow color range
lower_yellow = np.array([20, 70, 100], dtype=np.uint8)
upper_yellow = np.array([70, 255, 255], dtype=np.uint8)

# 빨간색 범위 1 (낮은 Hue 값)
lower_red1 = np.array([0, 50, 50], dtype=np.uint8)
upper_red1 = np.array([10, 255, 255], dtype=np.uint8)

# 빨간색 범위 2 (높은 Hue 값)
lower_red2 = np.array([170, 50, 50], dtype=np.uint8)
upper_red2 = np.array([180, 255, 255], dtype=np.uint8)

class AgvStep(Enum):
    Driving = 1
    Parking_Step1 = 2
    Parking_Step1_End = 3
    Parking_Step2 = 4
    Stop = 5

class AgvNode(Node):
    def __init__(self):
        super().__init__("Agv_Node")
        self.get_logger().info("Agv_Node has been started..")
        self.is_agv_ready = False

        self.agv_ready_publisher = self.create_publisher(Bool, "agv_ready", 10)
        self.plc_complate_subscription = self.create_subscription(Bool, "plc_complate", self.plc_complate_callback, 10)

        self.cap = cv2.VideoCapture(0)
        self.agv = MyAgv("/dev/ttyAMA2", 115200)

        self._running = True
        self.agvStep = AgvStep.Driving
        self.agv.set_led(2, 0, 0, 255)

        self.camera_thread = threading.Thread(target=self.camera_thread, daemon=True)
        self.agv_controll_thread = threading.Thread(target=self.agv_controll_thread, daemon=True)

        self.camera_thread.start()
        time.sleep(5)
        self.agv_controll_thread.start()
    
    def calculate_pid(self, error, prev_error, integral, prev_output=0):
        global Kp, Ki, Kd

        derivative = error - prev_error
        output = Kp * error + Ki * integral + Kd * derivative
        output = max(min(int(output), 127), -127)  # Clamp output
        smoothed_output = 0.7 * prev_output + 0.3 * output  # 스무딩 강화
        return int(smoothed_output)  # Convert to integer
    
    def calculate_turn_speed(self, pid_output):
        min_turn_speed = 20  # 최소 회전 속도 (조금 더 큰 값으로 설정)
        max_turn_speed = 100  # 최대 회전 속도 (더 큰 값으로 설정)
        scale_factor = 1.1   # 회전 속도 증폭 비율 추가
        # PID 출력값에 따라 선형 비례 계산 (증폭 적용)
        turn_speed = min_turn_speed + (max_turn_speed - min_turn_speed) * abs(pid_output) / 127
        return int(turn_speed * scale_factor)

    def process_frame(self, roi):
        global direction, prev_direction, pid_output, lower_yellow, upper_yellow, lower_red1, upper_red1, prev_error, integral, upper_red2, upper_red2, error, diff_ang, angle_prev_error, angle_integral, direction_lock
        # height, _, _= frame.shape

        # roi_height = int(height / 6)  # Adjust height of the ROI as needed
        # roi_top = height - roi_height
        # roi = frame[roi_top:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 모폴로지 연산에 사용할 커널 정의
        kernel = np.ones((5, 5), np.uint8)

        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        red_mask = red_mask1 | red_mask2
        
        yellow_detect = cv2.countNonZero(yellow_mask) > 100
        red_detect = cv2.countNonZero(red_mask) > 100

        roi_h, roi_w = roi.shape[:2]
        # ROI의 중심 세로선을 빨간색으로 그립니다.
        cv2.line(roi, (roi_w // 2, 0), (roi_w // 2, roi_h), (250, 0, 0), 2)
        
        if red_detect:
            #정지선 만남 Dirving상태였다면 Parking_Step1 돌입
            if self.agvStep == AgvStep.Driving:
                self.agvStep = AgvStep.Parking_Step1
                self.agv.set_led(2, 255, 0, 0)
                parking_step1_thread = threading.Thread(target=self.parking_step1, daemon=True)
                parking_step1_thread.start()
        elif yellow_detect:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, binary_image = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
            yellow_binary_image = cv2.bitwise_and(binary_image, binary_image, mask=yellow_mask)
            yellow_contours, _ = cv2.findContours(yellow_binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(yellow_contours) >= 1:
                max_contour = max(yellow_contours, key=cv2.contourArea)
                M = cv2.moments(max_contour)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    #cy = int(M["m01"] / M["m00"])

                    center_points = []
                    for y in range(roi_h):
                        # 해당 행에서 흰색 픽셀(노란색 영역) 인덱스 찾기
                        xs = np.where(yellow_mask[y, :] > 0)[0]
                        if len(xs) > 0:
                            # 흰색 픽셀의 평균 x 좌표 계산
                            x_avg = int(np.mean(xs))
                            center_points.append((x_avg, y))

                    # 충분한 점이 있다면, 이 점들을 선으로 이어서 그리기
                    if len(center_points) > 1:
                        pts = np.array(center_points, np.int32)
                        pts = pts.reshape((-1, 1, 2))

                        #두 선의 각도
                        line = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
                        vx, vy, x0, y0 = line.flatten()

                        # ROI 높이 roi_h 가정, y=0과 y=roi_h에서의 두 점을 계산
                        t0 = (0 - y0) / vy if vy != 0 else 0
                        t1 = (roi_h - y0) / vy if vy != 0 else 0

                        x_top = x0 + vx * t0
                        x_bottom = x0 + vx * t1

                        # 계산된 두 점을 사용하여 직선을 그림 (여기서는 파란색 선, 두께 2)
                        cv2.line(roi, (int(x_top), 0), (int(x_bottom), roi_h), (255, 0, 0), 2)

                        # 이미 x_top, x_bottom, roi_h가 계산되어 있음
                        dx = x_bottom - x_top
                        dy = roi_h
                        blue_line_angle = math.degrees(math.atan2(dx, dy))  # 수직 기준 상대 각도

                    frame_center = yellow_binary_image.shape[1] // 2
                    error = cx - frame_center

                    with direction_lock:
                        #diff_ang = blue_line_angle  # 두 선 사이의 각도 차이
                        pid_output = self.calculate_pid(error, prev_error, integral)
                        prev_error = error
                        integral += error
                        integral = max(min(integral, 100), -100)  # Clamp integral value

                        diff_ang = self.calculate_pid(blue_line_angle, angle_prev_error, angle_integral)
                        angle_prev_error = blue_line_angle
                        angle_integral += blue_line_angle
                        angle_integral = max(min(angle_integral, 100), -100)  # Clamp integral value

                        cv2.putText(roi, f"pid_output : {pid_output}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)
                        cv2.putText(roi, f"diff_ang : {diff_ang}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)

                        if self.agvStep == AgvStep.Driving:
                            if pid_output > 3: # Turn right
                                if prev_direction != "RIGHT":
                                    self.agv.stop()
                                direction = "RIGHT"
                            elif pid_output < -3:  # Turn left
                                if prev_direction != "LEFT":
                                    self.agv.stop()
                                direction = "LEFT"
                            else:
                                if prev_direction != "FOWARD":
                                    self.agv.stop() 
                                direction = "FOWARD"
                    
                    if self.agvStep == AgvStep.Parking_Step1_End:
                        self.agvStep = AgvStep.Parking_Step2
                        parking_step2_thread = threading.Thread(target=self.parking_step2, daemon=True)
                        parking_step2_thread.start()
        else:
            direction = None
            if self.agvStep == AgvStep.Parking_Step2:
                with direction_lock:
                    pid_output = -100
                    diff_ang = -100


        cv2.imshow("roi", roi)  
        cv2.waitKey(1)  # GUI 이벤트 처리 및 창 업데이트

    def camera_thread(self):
        global mtx, dist
        while self._running:                                                                        
            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().info("Camera error..")
                break
            frame = cv2.undistort(frame, mtx, dist)
            height, width, _ = frame.shape

            # 관심 영역(ROI) 추출 (예: 하단 1/6 영역)
            roi_height = int(height / 6)
            roi_top = height - roi_height
            roi = frame[roi_top:, :]
            roi_h, roi_w = roi.shape[:2]

            src_pts = np.float32([
                [int(roi_w * 0.25), 0],      # 좌측 상단 0.25
                [int(roi_w * 0.75), 0],        # 우측 상단 0.75
                [roi_w, roi_h],              # 우측 하단
                [0, roi_h]                   # 좌측 하단
            ])

            # dst_pts: 보정 후 정면 뷰의 네 점 (직사각형)
            dst_pts = np.float32([
                [0, 0],
                [roi_w, 0],
                [roi_w, roi_h],
                [0, roi_h]
            ])

            H_matrix, _ = cv2.findHomography(src_pts, dst_pts)
            corrected_roi = cv2.warpPerspective(roi, H_matrix, (roi_w, roi_h))

            self.process_frame(corrected_roi)
            time.sleep(0.03)

    def parking_step1(self):
        global base_speed
        #마지막 까지 왔다 뒤로 돌아서 주차 하자
        self.agv.stop()
        time.sleep(1)
        self.agv.go_ahead(1, timeout=2.0)
        time.sleep(1)
        for i in range(50):
            self.agv._mesg(base_speed, base_speed, base_speed+10)
            time.sleep(0.1)
        
        self.agv.stop()
        self.agvStep = AgvStep.Parking_Step1_End
    
    def parking_step2(self):
        global base_speed, diff_ang, pid_output, direction_lock
        threshold_distance = 2  # 노란색 영역 중심과의 허용 오차 (픽셀 단위)
        threshold_angle = 1     # 각도 오차 허용 범위 (도 단위)
        max_attempts = 10        # 최대 조정 시도 횟수
        angle_attempt = 0
        distance_attempt = 0

        self.get_logger().info("[INFO] Aligning AGV to center of the yellow area...")
        time.sleep(5)

        while angle_attempt < max_attempts:
            with direction_lock:
                #local_distance = pid_output    # 양수: AGV 기준선보다 노란색 영역이 왼쪽에 있음
                local_diff_ang = int(diff_ang)    # AGV 기준(수평)과 노란색 중심선의 각도 차이

            if local_diff_ang == -100:
                self.agv.clockwise_rotation(1, timeout=0.5)
                time.sleep(5)
                continue

            self.get_logger().info(f"[DEBUG] Attempt {angle_attempt+1}: Angle error: {local_diff_ang}")   #Distance error: {local_distance},

            # 만약 좌우 오차와 각도 오차 모두 허용범위 내라면 정렬 완료
            if abs(local_diff_ang) <= threshold_angle:   #abs(local_distance) <= threshold_distance and 
                self.get_logger().info("[INFO] Angle Center Complate!")
                break

            # 1. 각도 보정: AGV의 진행 방향(수평)과 노란색 영역 중심선의 각도가 많이 차이날 경우
            if abs(local_diff_ang) > threshold_angle:
                if local_diff_ang > 0:
                    # 예: 노란색 중심선이 오른쪽으로 기울어져 있으면, AGV가 좌측으로 회전하여 보정
                    self.get_logger().info("[INFO] Rotating AGV counterclockwise for alignment")
                    self.agv.counterclockwise_rotation(1, timeout=0.2)
                elif local_diff_ang < 0:
                    self.get_logger().info("[INFO] Rotating AGV clockwise for alignment")
                    self.agv.clockwise_rotation(1, timeout=0.2)
            angle_attempt += 1
            time.sleep(5)

        while distance_attempt < max_attempts:
            with direction_lock:
                local_distance = pid_output    # 양수: AGV 기준선보다 노란색 영역이 왼쪽에 있음
                #local_diff_ang = int(diff_ang)    # AGV 기준(수평)과 노란색 중심선의 각도 차이

            self.get_logger().info(f"[DEBUG] Attempt {distance_attempt+1}: distance error: {local_distance}")   #Distance error: {local_distance},

            if abs(local_distance) <= threshold_distance:   #abs(local_distance) <= threshold_distance and 
                self.get_logger().info("[INFO] Distance Center Complate!")
                break

            # 2. 좌우 위치 보정: 노란색 중심과의 수평 오차가 클 경우
            if abs(local_distance) > threshold_distance:
                if local_distance > 0:
                    # AGV 기준선보다 노란색 영역이 왼쪽에 있다면, AGV를 오른쪽으로 이동시켜야 함
                    self.get_logger().info("[INFO] Adjusting lateral position: Moving RIGHT")
                    self.agv.pan_right(1, timeout=0.3)
                elif local_distance < 0:
                    self.get_logger().info("[INFO] Adjusting lateral position: Moving LEFT")
                    self.agv.pan_left(1, timeout=0.3)

                distance_attempt += 1
                time.sleep(5)

        #self.get_logger().info("[WARN] Maximum alignment attempts reached. AGV may not be perfectly centered.")
        self.agv.stop()
        self.is_agv_ready = True

        #주차 완료 pub
        self.pub_agv_ready()
        self.agv.set_led(1, 255, 0, 0)

    def pub_agv_ready(self):
        msg = Bool()
        msg.data = self.is_agv_ready
        self.get_logger().info(f'pub: {msg.data}')

        self.agv_ready_publisher.publish(msg)

    def plc_complate_callback(self, msg):
        self.get_logger().info(f'sub: {msg.data}')
        data = msg.data
        
        if data is True and self.is_agv_ready is True:
            #PLC sub로 부터 완료 topic 받으면 해줘야 할 것들들
            self.is_agv_ready = False
            self.agvStep = AgvStep.Driving
            self.agv.set_led(2, 0, 0, 255)

    def agv_controll_thread(self):
        global pid_output, direction, prev_direction, error, base_speed
        no_detect_count = 0
        max_no_detect_count = 20

        while self._running:
            if self.agvStep != AgvStep.Driving:
                continue
            with direction_lock:
                local_pid_output = pid_output
                local_direction = direction
            turn_speed = int(self.calculate_turn_speed(local_pid_output))
            if local_pid_output < -8 and local_pid_output > 8:
                add_speed = turn_speed
            elif local_pid_output < -6 and local_pid_output > 6:
                add_speed = turn_speed // 2
            else:
                add_speed = turn_speed // 3

            if local_direction == "RIGHT":  # Turn right
                #print(f"Turning RIGHT with PID output: {pid_output}")
                self.agv._mesg(base_speed + add_speed, base_speed, base_speed - turn_speed)
                prev_direction = "RIGHT"
                no_detect_count = 0

            elif local_direction == "LEFT":  # Turn left
                #print(f"Turning LEFT with PID output: {pid_output}")
                self.agv._mesg(base_speed + add_speed, base_speed, base_speed + turn_speed)
                prev_direction = "LEFT"
                no_detect_count = 0
                
            elif local_direction == "FOWARD":  # Go straight
                #print("Moving FORWARD")
                self.agv._mesg(base_speed + 10, base_speed, base_speed)
                prev_direction = "FOWARD"
                no_detect_count = 0
            else:
                if prev_direction == "LEFT":
                    self.agv._mesg(base_speed, base_speed, base_speed + 10)
                    no_detect_count += 1
                    if no_detect_count >= max_no_detect_count:
                        self.agv.stop()
                        for i in range(max_no_detect_count):
                            self.agv._mesg(base_speed, base_speed, base_speed - 10)
                            time.sleep(0.1)
                        self.agv.stop()
                        no_detect_count = 0
                        prev_direction = None
                elif prev_direction == "RIGHT":
                    self.agv._mesg(base_speed, base_speed, base_speed - 10)
                    no_detect_count += 1
                    if no_detect_count >= max_no_detect_count:
                        self.agv.stop()
                        for i in range(max_no_detect_count):
                            self.agv._mesg(base_speed, base_speed, base_speed + 10)
                            time.sleep(0.1)
                        self.agv.stop()
                        prev_direction = None
                        no_detect_count = 0
                elif direction is None:  # Only stop if AGV was moving
                    self.agv.stop()
                    prev_direction = None
            time.sleep(0.1)

    def destory_node(self):
        self._running = False
        if self.camera_thread.is_alive():
            self.camera_thread.join()
        if self.agv_controll_thread.is_alive():
            self.agv_controll_thread.join()

        super().destroy_node()
        self.agv.stop()
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    agv_node = AgvNode()

    try:
        rclpy.spin(agv_node)
    except KeyboardInterrupt:
        agv_node.get_logger().info("KeyboardInterrupt Stop.")
    finally:
        agv_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()