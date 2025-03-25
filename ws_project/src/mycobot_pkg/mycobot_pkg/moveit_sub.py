#!/usr/bin/env python3
# Python 3 스크립트 실행을 위한 셔뱅(Shebang) 라인

import rclpy, math  # ROS 2 Python 클라이언트 라이브러리와 수학 연산 라이브러리 가져오기
from rclpy.node import Node  # ROS 2 노드 클래스 가져오기
from sensor_msgs.msg import JointState  # 관절 상태 메시지 타입 가져오기
from pymycobot.mycobot320 import MyCobot320  # MyCobot 320 로봇 제어 라이브러리 가져오기
import numpy as np  # 수치 계산을 위한 NumPy 라이브러리 가져오기
import time  # 시간 관련 작업을 위한 라이브러리 가져오기

class topic_to_angle(Node):  # ROS 2 노드를 상속받는 커스텀 노드 클래스 정의
    def __init__(self):  # 클래스 초기화 메서드
        super().__init__("topic_to_angle")  # 부모 클래스(Node) 초기화, 노드 이름 설정
        self.get_logger().info("the node has started")  # 노드 시작 로그 출력
        self.mc = MyCobot320("/dev/ttyACM0", 115200)  # MyCobot 320 로봇 객체 생성 (USB 포트와 통신 속도 설정)
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 30)  # 로봇 모든 관절을 0도로 초기화 (속도 30)
        self.sub_ = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 1)  # joint_states 토픽 구독 설정
    
    def joint_state_callback(self, msg):  # 관절 상태 메시지 수신 시 호출되는 콜백 메서드
        angles = [round(math.degrees(r)) for r in msg.position]  # 수신된 관절 위치를 각도(도)로 변환 및 반올림
        temp = angles.pop(4)  # 4번 인덱스의 각도를 임시 변수에 저장하고 제거
        angles.insert(1, temp)  # 임시 변수의 각도를 1번 인덱스에 삽입
        gripper_raw = angles.pop()  # 마지막 각도(그리퍼)를 raw 값으로 추출
        gripper = round((-gripper_raw / 46) * 100)  # 그리퍼 각도를 0-100 사이 값으로 변환
        self.get_logger().info(f"angle: {angles}, gripper: {gripper}")  # 변환된 각도와 그리퍼 값 로그 출력
        self.mc.send_angles(angles, 30)  # 계산된 각도로 로봇 관절 이동 (속도 30)
        self.mc.set_gripper_value(gripper, 20)  # 계산된 그리퍼 값으로 그리퍼 제어 (속도 20)

def main(args=None):  # 메인 함수 정의
    rclpy.init(args = args)  # ROS 2 클라이언트 라이브러리 초기화
    node = topic_to_angle()  # topic_to_angle 노드 인스턴스 생성
    rclpy.spin(node)  # 노드 실행 (메시지 수신 대기)
    rclpy.destroy_node(node)  # 노드 제거
    rclpy.shutdown()  # ROS 2 클라이언트 라이브러리 종료

if __name__ == "__main__":  # 스크립트가 직접 실행될 때만 메인 함수 호출
    main()
