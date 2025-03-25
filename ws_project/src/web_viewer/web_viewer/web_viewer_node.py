#!/usr/bin/env python3  # 이 스크립트를 Python 3로 실행하도록 지정

import rclpy  # ROS 2의 Python 클라이언트 라이브러리인 rclpy를 임포트
from rclpy.node import Node  # ROS 2 노드 클래스를 임포트
from std_msgs.msg import Bool  # ROS 2의 표준 Bool 메시지 타입을 임포트
# kairos_interfaces 패키지에서 PackagesStatusMsg 임포트
from kairos_interfaces.msg import PackageStatusMsg  # 사용자 정의 메시지 타입 PackageStatusMsg를 임포트
import json  # JSON 데이터 처리를 위한 Python 표준 라이브러리 임포트
from flask import Flask, render_template, Response, request, jsonify  # 웹 서버 구축을 위한 Flask 모듈 임포트
import threading  # 멀티스레딩 기능을 위한 Python 표준 라이브러리 임포트
import time  # 시간 관련 기능을 위한 Python 표준 라이브러리 임포트
from datetime import datetime  # 날짜와 시간 처리를 위한 Python 표준 라이브러리 임포트
import os  # 파일 경로 및 운영체제 관련 기능을 위한 Python 표준 라이브러리 임포트
import pkg_resources  # 패키지 리소스 관리(사용되지 않음)를 위한 라이브러리 임포트
from ament_index_python.packages import get_package_share_directory  # ROS 2 패키지의 공유 디렉토리 경로를 가져오는 함수 임포트

# 웹 서버 설정
app = Flask(__name__)  # Flask 애플리케이션 인스턴스 생성 (__name__은 현재 모듈 이름)

# 각 토픽의 데이터를 저장할 전역 변수
topic_data = {  # 토픽 데이터를 저장하는 전역 딕셔너리 초기화
    'package_data': {'package_id': 0, 'region': 'N/A', 'process': 0, 'process_text': 'N/A', 'last_updated': ''},  # 패키지 데이터 초기값 설정
    'agv_status': {'status': 'N/A', 'status_bool': False, 'last_updated': ''},  # AGV 상태 데이터 초기값 설정
    'plc_status': {'status': 'N/A', 'status_bool': False, 'last_updated': ''}  # PLC 상태 데이터 초기값 설정
}

# 패키지 로그를 저장할 배열
package_logs = []  # 패키지 처리 로그를 저장하는 전역 리스트 초기화

# ROS2 노드 클래스 정의
class WebViewerNode(Node):  # WebViewerNode 클래스를 정의 (Node 클래스 상속)
    def __init__(self):  # 클래스 초기화 메서드 정의
        super().__init__('web_viewer_node')  # 상위 클래스(Node)의 초기화 호출, 노드 이름 지정
        
        # 현재 처리 중인 패키지 ID
        self.current_package_id = None  # 현재 처리 중인 패키지 ID를 저장하는 변수 초기화 (None)
        
        # PackagesStatusMsg 토픽 구독
        self.package_sub = self.create_subscription(  # '/packages_status' 토픽 구독 설정
            PackageStatusMsg,  # 구독할 메시지 타입 지정
            '/packages_status',  # 구독할 토픽 이름 지정
            self.package_callback,  # 메시지 수신 시 호출할 콜백 함수 지정
            10)  # 큐 크기 설정 (10)
            
        # AGV 및 PLC 토픽 구독 (Bool 타입으로 수정)
        self.agv_ready_sub = self.create_subscription(  # '/agv_ready' 토픽 구독 설정
            Bool,  # 구독할 메시지 타입 지정 (Bool)
            '/agv_ready',  # 구독할 토픽 이름 지정
            self.agv_ready_callback,  # 메시지 수신 시 호출할 콜백 함수 지정
            10)  # 큐 크기 설정 (10)
            
        self.plc_complete_sub = self.create_subscription(  # '/plc_complate' 토픽 구독 설정
            Bool,  # 구독할 메시지 타입 지정 (Bool)
            '/plc_complate',  # 구독할 토픽 이름 지정
            self.plc_complete_callback,  # 메시지 수신 시 호출할 콜백 함수 지정
            10)  # 큐 크기 설정 (10)
        
        self.get_logger().info('Web Viewer node has been initialized')  # 노드 초기화 완료 로그 출력

    # PackagesStatusMsg 콜백 함수
    def package_callback(self, msg):  # '/packages_status' 토픽 메시지 수신 시 호출되는 콜백 함수
        global topic_data, package_logs  # 전역 변수 topic_data와 package_logs 사용 선언
        
        # Process 값에 따른 텍스트 설정
        process_text = "Unknown"  # 기본 프로세스 텍스트를 "Unknown"으로 설정
        if msg.process == 1:  # 메시지의 process 값이 1이면
            process_text = "픽업 포인트 접근 중"  # 프로세스 텍스트를 "픽업 포인트 접근 중"으로 설정
        elif msg.process == 2:  # 메시지의 process 값이 2이면
            process_text = "배치 포인트 접근 중"  # 프로세스 텍스트를 "배치 포인트 접근 중"으로 설정
        elif msg.process == 3:  # 메시지의 process 값이 3이면
            process_text = "배치 작업 완료"  # 프로세스 텍스트를 "배치 작업 완료"로 설정
            
        # 패키지 정보 업데이트
        topic_data['package_data'] = {  # topic_data의 'package_data' 항목 업데이트
            'package_id': msg.package_id,  # 패키지 ID 설정
            'region': msg.region,  # 지역 설정
            'process': msg.process,  # 프로세스 상태 설정
            'process_text': process_text,  # 프로세스 텍스트 설정
            'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # 마지막 업데이트 시간 설정
        }
        
        # 패키지 ID가 변경되었거나 처리 상태가 변경된 경우 로그 업데이트
        if msg.package_id != 0 and msg.package_id != self.current_package_id:  # 패키지 ID가 0이 아니고 현재 ID와 다를 경우
            self.current_package_id = msg.package_id  # 현재 패키지 ID 업데이트
            
            # 로그에 추가하기 위한 데이터 생성
            log_entry = {  # 새로운 로그 항목 생성
                'id': msg.package_id,  # 패키지 ID
                'region': msg.region,  # 지역
                'process': msg.process,  # 프로세스 상태
                'process_text': process_text,  # 프로세스 텍스트
                'timestamp': datetime.now().isoformat(),  # ISO 형식의 타임스탬프
                'formatted_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # 포맷된 시간
            }
            
            # 이미 존재하는 로그인지 확인
            existing_log = next((log for log in package_logs if log['id'] == msg.package_id), None)  # 동일 ID의 기존 로그 찾기
            
            if existing_log:  # 기존 로그가 존재하면
                # 기존 로그 업데이트
                for key, value in log_entry.items():  # 로그 항목의 모든 키-값 쌍을 순회
                    existing_log[key] = value  # 기존 로그 업데이트
            else:  # 기존 로그가 없으면
                # 새 로그 추가
                package_logs.append(log_entry)  # 새로운 로그 항목을 package_logs에 추가
                
                # 로그가 너무 많으면 오래된 로그 삭제 (최대 100개 유지)
                if len(package_logs) > 100:  # 로그 수가 100개를 초과하면
                    package_logs.pop(0)  # 가장 오래된 로그 삭제
        
        # 현재 패키지의 처리 상태만 변경된 경우
        elif msg.package_id == self.current_package_id:  # 메시지의 패키지 ID가 현재 ID와 같으면
            # 로그에서 현재 패키지 찾기
            for log in package_logs:  # package_logs를 순회
                if log['id'] == msg.package_id:  # 현재 패키지 ID와 일치하는 로그를 찾으면
                    log['process'] = msg.process  # 프로세스 상태 업데이트
                    log['process_text'] = process_text  # 프로세스 텍스트 업데이트
                    log['timestamp'] = datetime.now().isoformat()  # 타임스탬프 업데이트
                    log['formatted_time'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # 포맷된 시간 업데이트
                    break  # 루프 종료
        
        self.get_logger().info(f'Received package status - ID: {msg.package_id}, Region: {msg.region}, Process: {msg.process} ({process_text})')  # 패키지 상태 로그 출력
        
    # AGV Ready 콜백 함수 (Bool 타입으로 수정)
    def agv_ready_callback(self, msg):  # '/agv_ready' 토픽 메시지 수신 시 호출되는 콜백 함수
        global topic_data  # 전역 변수 topic_data 사용 선언
        status_text = 'AGV 도착' if msg.data else 'AGV 대기 중'  # Bool 값에 따라 상태 텍스트 설정
        
        topic_data['agv_status'] = {  # topic_data의 'agv_status' 항목 업데이트
            'status': status_text,  # 상태 텍스트 설정
            'status_bool': msg.data,  # Bool 값 설정
            'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # 마지막 업데이트 시간 설정
        }

        topic_data['plc_status'] = {  # topic_data의 'plc_status' 항목 업데이트
            'status': '컨베이어 작동 중',  # 상태 텍스트 설정
            'status_bool': False,  # Bool 값 설정
            'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # 마지막 업데이트 시간 설정
        }
        
        self.get_logger().info(f'Received AGV Ready signal: {msg.data} - {status_text}')  # AGV 상태 로그 출력
        
    # PLC Complete 콜백 함수 (Bool 타입으로 수정)
    def plc_complete_callback(self, msg):  # '/plc_complate' 토픽 메시지 수신 시 호출되는 콜백 함수
        global topic_data  # 전역 변수 topic_data 사용 선언
        status_text = '컨베이어 정지' if msg.data else '컨베이어 작동 중'  # Bool 값에 따라 상태 텍스트 설정
        
        topic_data['plc_status'] = {  # topic_data의 'plc_status' 항목 업데이트
            'status': status_text,  # 상태 텍스트 설정
            'status_bool': msg.data,  # Bool 값 설정
            'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # 마지막 업데이트 시간 설정
        }

        topic_data['agv_status'] = {  # topic_data의 'agv_status' 항목 업데이트
            'status': 'AGV 상차 완료 후 출발',  # 상태 텍스트 설정
            'status_bool': False,  # Bool 값 설정
            'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # 마지막 업데이트 시간 설정
        }
        self.get_logger().info(f'Received PLC Complete signal: {msg.data} - {status_text}')  # PLC 상태 로그 출력

# Flask 라우트 설정
@app.route('/')  # 루트 경로('/')에 대한 Flask 라우트 정의
def index():  # 루트 경로 요청 시 호출되는 함수
    # web_viewer 패키지의 config 폴더에서 HTML 템플릿 제공
    try:  # 예외 처리를 위한 try 블록 시작
        # package_path = get_package_share_directory('web_viewer')  # 패키지 공유 디렉토리 경로 가져오기 (주석 처리됨)
        template_path = os.path.join('web_page2.html')  # HTML 파일 경로 설정 (상대 경로)
        with open(template_path, 'r') as file:  # HTML 파일을 읽기 모드로 열기
            html_content = file.read()  # HTML 파일 내용 읽기
        return html_content  # 읽은 HTML 내용을 반환
    except Exception as e:  # 예외 발생 시 처리
        return f"Error loading template: {str(e)}"  # 에러 메시지 반환

@app.route('/topic_data')  # '/topic_data' 경로에 대한 Flask 라우트 정의
def get_topic_data():  # '/topic_data' 요청 시 호출되는 함수
    global topic_data, package_logs  # 전역 변수 topic_data와 package_logs 사용 선언
    response_data = {  # 응답 데이터를 담을 딕셔너리 생성
        'package_data': topic_data['package_data'],  # 패키지 데이터 추가
        'agv_status': topic_data['agv_status'],  # AGV 상태 데이터 추가
        'plc_status': topic_data['plc_status'],  # PLC 상태 데이터 추가
        'package_logs': package_logs  # 패키지 로그 데이터 추가
    }
    return json.dumps(response_data)  # JSON 형식으로 변환하여 반환

@app.route('/package_logs')  # '/package_logs' 경로에 대한 Flask 라우트 정의
def get_package_logs():  # '/package_logs' 요청 시 호출되는 함수
    global package_logs  # 전역 변수 package_logs 사용 선언
    return json.dumps(package_logs)  # 패키지 로그를 JSON 형식으로 변환하여 반환

@app.route('/clear_logs', methods=['POST'])  # '/clear_logs' 경로에 대한 Flask 라우트 정의 (POST 메서드)
def clear_logs():  # '/clear_logs' 요청 시 호출되는 함수
    global package_logs  # 전역 변수 package_logs 사용 선언
    package_logs = []  # 패키지 로그 리스트 초기화
    return jsonify({'success': True, 'message': '로그가 초기화되었습니다.'})  # 성공 메시지를 JSON으로 반환

# ROS2 스핀 함수를 별도 스레드로 실행
def ros_spin(node):  # ROS 노드를 스핀하는 함수 정의
    rclpy.spin(node)  # ROS 노드 스핀 실행 (메시지 처리 루프)
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS 2 종료

# 메인 함수
def main(args=None):  # 메인 함수 정의 (기본 인자 None)
    # ROS2 초기화
    rclpy.init(args=args)  # ROS 2 초기화
    node = WebViewerNode()  # WebViewerNode 인스턴스 생성
    
    # ROS2 스핀 스레드 시작
    ros_thread = threading.Thread(target=ros_spin, args=(node,))  # ROS 스핀을 위한 스레드 생성
    ros_thread.daemon = True  # 데몬 스레드로 설정 (메인 스레드 종료 시 함께 종료)
    ros_thread.start()  # 스레드 시작
    
    # Flask 서버 시작
    app.run(host='0.0.0.0', port=5000, debug=False)  # Flask 웹 서버 실행 (모든 IP에서 접근 가능, 포트 5000)
    
    # ROS2 종료 처리
    if rclpy.ok():  # ROS가 아직 실행 중이면
        rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':  # 스크립트가 직접 실행될 때만 아래 코드 실행
    main()  # 메인 함수 호출
