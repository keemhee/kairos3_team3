#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
# kairos_interfaces 패키지에서 PackagesStatusMsg 임포트
from kairos_interfaces.msg import PackageStatusMsg
import json
from flask import Flask, render_template, Response, request, jsonify
import threading
import time
from datetime import datetime
import os
import pkg_resources
from ament_index_python.packages import get_package_share_directory

# 웹 서버 설정
app = Flask(__name__)

# 각 토픽의 데이터를 저장할 전역 변수
topic_data = {
    'package_data': {'package_id': 0, 'region': 'N/A', 'process': 0, 'process_text': 'N/A', 'last_updated': ''},
    'agv_status': {'status': 'N/A', 'status_bool': False, 'last_updated': ''},
    'plc_status': {'status': 'N/A', 'status_bool': False, 'last_updated': ''}
}

# 패키지 로그를 저장할 배열
package_logs = []

# ROS2 노드 클래스 정의
class WebViewerNode(Node):
    def __init__(self):
        super().__init__('web_viewer_node')
        
        # 현재 처리 중인 패키지 ID
        self.current_package_id = None
        
        # PackagesStatusMsg 토픽 구독
        self.package_sub = self.create_subscription(
            PackageStatusMsg,
            '/packages_status',
            self.package_callback,
            10)
            
        # AGV 및 PLC 토픽 구독 (Bool 타입으로 수정)
        self.agv_ready_sub = self.create_subscription(
            Bool,
            '/agv_ready',
            self.agv_ready_callback,
            10)
            
        self.plc_complete_sub = self.create_subscription(
            Bool,
            '/plc_complate',
            self.plc_complete_callback,
            10)
        
        self.get_logger().info('Web Viewer node has been initialized')

    # PackagesStatusMsg 콜백 함수
    def package_callback(self, msg):
        global topic_data, package_logs
        
        # Process 값에 따른 텍스트 설정
        process_text = "Unknown"
        if msg.process == 1:
            process_text = "픽업 포인트 접근 중"
        elif msg.process == 2:
            process_text = "배치 포인트 접근 중"
        elif msg.process == 3:
            process_text = "배치 작업 완료"
            
        # 패키지 정보 업데이트
        topic_data['package_data'] = {
            'package_id': msg.package_id,
            'region': msg.region,
            'process': msg.process,
            'process_text': process_text,
            'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        # 패키지 ID가 변경되었거나 처리 상태가 변경된 경우 로그 업데이트
        if msg.package_id != 0 and msg.package_id != self.current_package_id:
            self.current_package_id = msg.package_id
            
            # 로그에 추가하기 위한 데이터 생성
            log_entry = {
                'id': msg.package_id,
                'region': msg.region,
                'process': msg.process,
                'process_text': process_text,
                'timestamp': datetime.now().isoformat(),
                'formatted_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            # 이미 존재하는 로그인지 확인
            existing_log = next((log for log in package_logs if log['id'] == msg.package_id), None)
            
            if existing_log:
                # 기존 로그 업데이트
                for key, value in log_entry.items():
                    existing_log[key] = value
            else:
                # 새 로그 추가
                package_logs.append(log_entry)
                
                # 로그가 너무 많으면 오래된 로그 삭제 (최대 100개 유지)
                if len(package_logs) > 100:
                    package_logs.pop(0)
        
        # 현재 패키지의 처리 상태만 변경된 경우
        elif msg.package_id == self.current_package_id:
            # 로그에서 현재 패키지 찾기
            for log in package_logs:
                if log['id'] == msg.package_id:
                    log['process'] = msg.process
                    log['process_text'] = process_text
                    log['timestamp'] = datetime.now().isoformat()
                    log['formatted_time'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    break
        
        self.get_logger().info(f'Received package status - ID: {msg.package_id}, Region: {msg.region}, Process: {msg.process} ({process_text})')
        
    # AGV Ready 콜백 함수 (Bool 타입으로 수정)
    def agv_ready_callback(self, msg):
        global topic_data
        status_text = 'AGV 도착' if msg.data else 'AGV 대기 중'
        
        topic_data['agv_status'] = {
            'status': status_text,
            'status_bool': msg.data,
            'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }

        topic_data['plc_status'] = {
            'status': '컨베이어 작동 중',
            'status_bool': False,
            'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        self.get_logger().info(f'Received AGV Ready signal: {msg.data} - {status_text}')
        
    # PLC Complete 콜백 함수 (Bool 타입으로 수정)
    def plc_complete_callback(self, msg):
        global topic_data
        status_text = '컨베이어 정지' if msg.data else '컨베이어 작동 중'
        
        topic_data['plc_status'] = {
            'status': status_text,
            'status_bool': msg.data,
            'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }

        topic_data['agv_status'] = {
            'status': 'AGV 상차 완료 후 출발',
            'status_bool': False,
            'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        self.get_logger().info(f'Received PLC Complete signal: {msg.data} - {status_text}')

# Flask 라우트 설정
@app.route('/')
def index():
    # web_viewer 패키지의 config 폴더에서 HTML 템플릿 제공
    try:
        # package_path = get_package_share_directory('web_viewer')
        template_path = os.path.join('web_page2.html')
        with open(template_path, 'r') as file:
            html_content = file.read()
        return html_content
    except Exception as e:
        return f"Error loading template: {str(e)}"

@app.route('/topic_data')
def get_topic_data():
    global topic_data, package_logs
    response_data = {
        'package_data': topic_data['package_data'],
        'agv_status': topic_data['agv_status'],
        'plc_status': topic_data['plc_status'],
        'package_logs': package_logs
    }
    return json.dumps(response_data)

@app.route('/package_logs')
def get_package_logs():
    global package_logs
    return json.dumps(package_logs)

@app.route('/clear_logs', methods=['POST'])
def clear_logs():
    global package_logs
    package_logs = []
    return jsonify({'success': True, 'message': '로그가 초기화되었습니다.'})

# ROS2 스핀 함수를 별도 스레드로 실행
def ros_spin(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# 메인 함수
def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    node = WebViewerNode()
    
    # ROS2 스핀 스레드 시작
    ros_thread = threading.Thread(target=ros_spin, args=(node,))
    ros_thread.daemon = True
    ros_thread.start()
    
    # Flask 서버 시작
    app.run(host='0.0.0.0', port=5000, debug=False)
    
    # ROS2 종료 처리
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()