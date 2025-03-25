import rclpy  # ROS 2의 Python 클라이언트 라이브러리를 임포트
from rclpy.node import Node  # ROS 2 노드 클래스를 임포트
from pymodbus.client import ModbusSerialClient as ModbusClient  # Modbus 통신을 위한 클라이언트 라이브러리 임포트
from pymodbus.exceptions import ModbusIOException  # Modbus IO 예외 처리를 위한 모듈 임포트
import time  # 시간 관련 기능을 사용하기 위해 임포트
import threading  # 스레드 처리를 위해 임포트
from enum import Enum  # 열거형(Enum)을 사용하기 위해 임포트
from kairos_interfaces.action import PlcCobot  # 사용자 정의 액션 인터페이스 임포트
from kairos_interfaces.srv import PackageInfo  # 사용자 정의 서비스 인터페이스 임포트
from rclpy.action import ActionClient  # ROS 2 액션 클라이언트 클래스를 임포트
from std_msgs.msg import Bool  # ROS 2 기본 메시지 타입(Bool) 임포트
from kairos_interfaces.msg import PackageStatusMsg  # 사용자 정의 메시지 타입 임포트

# 글로벌 변수 설정
SLAVE_ID_1 = 1  # 첫 번째 PLC 슬레이브 ID 설정
SLAVE_ID_2 = 2  # 두 번째 PLC 슬레이브 ID 설정
MAX_RETRIES = 2  # 최대 재시도 횟수 설정
prev_values = {200: None, 210: None, 220: None, 230: None, 240: None}  # 이전 PLC 레지스터 값 저장용 딕셔너리
prev_values_init = {130: None, 140: None, 150: None}  # 초기 설정값 저장용 딕셔너리
prev_inv_values = {"freq": 0, "accel": 0, "decel": 0}  # 인버터 이전 값 저장용 딕셔너리
prev_popup_status = False  # 이전 팝업 상태 저장용 변수
prev_send_addr5 = None  # 이전 5번 주소 값 저장용 변수

PORT = '/dev/ttyUSB0'  # Modbus 통신에 사용할 시리얼 포트 경로 설정 (예시)
BAUDRATE = 9600  # 통신 속도(보드레이트) 설정
PARITY = 'N'  # 패리티 설정 (None)
STOPBITS = 1  # 스톱 비트 설정
BYTESIZE = 8  # 데이터 비트 수 설정
TIMEOUT = 1  # 통신 타임아웃 시간(초) 설정

INIT_FRQ = 600  # 초기 주파수 값 설정
INIT_ACCEL = 10  # 초기 가속 시간 값 설정
INIT_DECEL = 10  # 초기 감속 시간 값 설정

TOTAL_PACKAGE = 10  # 처리할 총 패키지 수 설정

class PlcMode(Enum):  # PLC 모드를 정의하는 열거형 클래스
    Manual = 1  # 수동 모드
    Auto = 2  # 자동 모드

class PlcNode(Node):  # ROS 2 노드를 상속받아 PLC 노드 클래스 정의
    def __init__(self, plc_client):  # 생성자 정의, plc_client 인자 받음
        super().__init__("Plc_Node")  # 부모 클래스(Node)의 생성자 호출, 노드 이름 설정
        self.get_logger().info("Plc Node has been Started...")  # 노드 시작 로그 출력

        self.agv_ready_subscription = self.create_subscription(Bool, "agv_ready", self.agv_ready_callback, 10)  # AGV 준비 상태 구독자 생성
        self.plc_complate_publisher = self.create_publisher(Bool, "plc_complate", 10)  # PLC 완료 상태 발행자 생성
        self.package_info_service_client = self.create_client(PackageInfo, "package_info")  # 패키지 정보 서비스 클라이언트 생성
        self.plc_client_ = ActionClient(self, PlcCobot, 'pick_place_action')  # PLC 코봇 액션 클라이언트 생성
        self.package_del_publisher = self.create_publisher(Bool, 'package_delete', 10)  # 패키지 삭제 발행자 생성
        self.cobot_init_publisher = self.create_publisher(Bool, 'cobot_init', 10)  # 코봇 초기화 발행자 생성
        self.packages_status = self.create_publisher(PackageStatusMsg, 'packages_status', 10)  # 패키지 상태 발행자 생성

        self.is_cobot_move = False  # 코봇 이동 상태 변수 초기화
        self.cobot_action_complated_count = 0  # 코봇 액션 완료 횟수 변수 초기화
        self.is_plc_ready_value = False  # PLC 준비 상태 변수 초기화

        self.plc_client = plc_client  # PLC 클라이언트 객체 저장
        self.Mode = PlcMode.Manual  # 초기 모드를 수동으로 설정
        self.is_move = False  # 이동 상태 변수 초기화
        self.thread_running = True  # 스레드 실행 상태 변수 초기화

        self.is_sensor = False  # 센서 상태 변수 초기화
        self.is_agv_ready = False  # AGV 준비 상태 변수 초기화
        self.initiate_setting()  # 초기 설정 함수 호출

        self.thread = threading.Thread(target=self.plc_thread, daemon=True)  # PLC 스레드 생성 (데몬 스레드로 설정)
        self.thread.start()  # 스레드 시작

    def initiate_setting(self):  # PLC 초기 설정 함수
        self.write_register(120, 1, SLAVE_ID_1)  # 120번 주소에 1을 써서 정지 신호 보냄 (SLAVE_ID_1)
        self.write_register(5, 5057, SLAVE_ID_2)  # 5번 주소에 5057을 써서 인버터 정지 (SLAVE_ID_2)
        self.write_register(120, 0, SLAVE_ID_1)  # 120번 주소에 0을 써서 정지 초기화 (SLAVE_ID_1)
        self.write_register(10, 1, SLAVE_ID_1)  # 10번 주소에 1을 써서 수동 모드 설정 (SLAVE_ID_1)
        self.write_register(4, INIT_FRQ, SLAVE_ID_2)  # 4번 주소에 초기 주파수 값 쓰기 (SLAVE_ID_2)
        self.write_register(6, INIT_ACCEL, SLAVE_ID_2)  # 6번 주소에 초기 가속 시간 값 쓰기 (SLAVE_ID_2)
        self.write_register(7, INIT_DECEL, SLAVE_ID_2)  # 7번 주소에 초기 감속 시간 값 쓰기 (SLAVE_ID_2)

        time.sleep(1)  # 1초 대기

        prev_inv_values["freq"] = INIT_FRQ  # 이전 주파수 값 업데이트
        prev_inv_values["accel"] = INIT_ACCEL  # 이전 가속 시간 값 업데이트
        prev_inv_values["decel"] = INIT_DECEL  # 이전 감속 시간 값 업데이트

        print(f"Read values - Frq: {INIT_FRQ}, Accel: {INIT_ACCEL}, Decel: {INIT_DECEL}")  # 초기 값 출력

        # 매핑된 주소 값 정의
        new_values_init = {
            130: INIT_FRQ,  # 인버터 현재 주파수 주소에 초기 주파수 값 설정
            140: INIT_ACCEL,  # 인버터 현재 가속 시간 주소에 초기 가속 시간 값 설정
            150: INIT_DECEL,  # 인버터 현재 감속 시간 주소에 초기 감속 시간 값 설정
        }

        # 변경이 발생한 경우에만 값 쓰기
        for index, (addr, new_value) in enumerate(new_values_init.items()):  # 각 주소와 값 순회
            if prev_values_init[addr] != new_value:  # 이전 값과 다를 경우에만 실행
                self.write_register(addr, new_value, SLAVE_ID_1)  # 해당 주소에 새 값 쓰기 (SLAVE_ID_1)
                self.write_register(50 + index, new_value, SLAVE_ID_1)  # 50번대 주소에도 값 쓰기 (SLAVE_ID_1)
                prev_values_init[addr] = new_value  # 이전 값 업데이트

    def agv_ready_callback(self, msg):  # AGV 준비 상태 메시지 콜백 함수
        self.get_logger().info(f'sub: {msg.data}')  # 수신된 메시지 로그 출력
        # Manual Mode일 경우 바로 반환
        if self.Mode == PlcMode.Manual:  # 현재 모드가 수동이면
            return  # 함수 종료

        if msg.data is True:  # AGV 준비 상태가 True라면
            self.is_agv_ready = True  # AGV 준비 상태 변수 설정
            self.cobot_action_complated_count = 0  # 코봇 액션 완료 횟수 초기화

    def call_packageinfo_service(self):  # 패키지 정보 서비스 호출 함수
        self.is_cobot_move = True  # 코봇 이동 상태를 True로 설정
        request = PackageInfo.Request()  # 서비스 요청 객체 생성
        self.get_logger().info(f'Send Service')  # 서비스 요청 로그 출력
        future = self.package_info_service_client.call_async(request)  # 비동기 서비스 호출
        future.add_done_callback(self.packageinfo_response_callback)  # 응답 완료 시 호출할 콜백 함수 등록

    def packageinfo_response_callback(self, future):  # 패키지 정보 서비스 응답 콜백 함수
        try:  # 예외 처리 시작
            response = future.result()  # 서비스 응답 결과 가져오기
            self.get_logger().info(f'Result: {response}')  # 응답 결과 로그 출력

            action_request = PlcCobot.Goal()  # 액션 목표 객체 생성
            action_request.is_plc_ready = True  # PLC 준비 상태 설정
            action_request.package_id = response.package_id  # 패키지 ID 설정
            action_request.region = response.region  # 지역 설정
            action_request.width = response.width  # 너비 설정
            action_request.height = response.height  # 높이 설정
            action_request.depth = response.depth  # 깊이 설정
            action_request.process = response.process  # 프로세스 설정
            action_request.x = response.x  # x 좌표 설정
            action_request.y = response.y  # y 좌표 설정
            action_request.theta = response.theta  # theta 값 설정

            # 지역에 따른 로그 데이터 설정
            logdata = 0  # 로그 데이터 초기화
            if response.region == "서울":  # 지역이 서울이면
                logdata = 3  # 로그 데이터 3 설정
            elif response.region == "경기":  # 지역이 경기이면
                logdata = 5  # 로그 데이터 5 설정
            elif response.region == "부산":  # 지역이 부산이면
                logdata = 7  # 로그 데이터 7 설정

            self.write_register(234, logdata, SLAVE_ID_1)  # 234번 주소에 로그 데이터 쓰기 (SLAVE_ID_1)
            self.write_register(123, 1, SLAVE_ID_1)  # 123번 주소에 1 쓰기 (SLAVE_ID_1)
            time.sleep(0.1)  # 0.1초 대기
            self.write_register(123, 0, SLAVE_ID_1)  # 123번 주소에 0 쓰기 (SLAVE_ID_1)
            self.send_request(action_request)  # 액션 요청 전송

        except Exception as e:  # 예외 발생 시
            self.get_logger().warn(f'{e}')  # 예외 로그 출력

    # Action Client 함수 정의
    def send_request(self, action_request):  # 액션 요청 전송 함수
        self.get_logger().info(f'Sending goal')  # 목표 전송 로그 출력
        self.plc_client_.wait_for_server()  # 액션 서버가 준비될 때까지 대기
        #self.get_logger().info(f'Action server is available')  # 서버 준비 로그 (주석 처리됨)

        future = self.plc_client_.send_goal_async(action_request, feedback_callback=self.plc_feedback_callback)  # 비동기 목표 전송
        #self.get_logger().info(f'Goal sent, waiting for response...')  # 목표 전송 완료 로그 (주석 처리됨)
        future.add_done_callback(self.goal_response_callback)  # 목표 응답 완료 시 호출할 콜백 함수 등록

    def goal_response_callback(self, future):  # 액션 목표 응답 콜백 함수
        """서버에서 목표 수락 여부 확인"""
        goal_handle = future.result()  # 목표 처리 결과 가져오기

        if not goal_handle.accepted:  # 목표가 수락되지 않았다면
            self.get_logger().info('Goal rejected')  # 목표 거부 로그 출력
            return  # 함수 종료

        self.get_logger().info('Goal accepted')  # 목표 수락 로그 출력

        # 결과 요청 및 콜백 함수 등록
        _get_result_future = goal_handle.get_result_async()  # 비동기 결과 요청
        _get_result_future.add_done_callback(self.plc_result_callback)  # 결과 완료 시 호출할 콜백 함수 등록

    def plc_feedback_callback(self, feedback_msg):  # 액션 피드백 콜백 함수
        """서버에서 실시간 피드백을 받을 때 실행"""
        feedback = feedback_msg.feedback  # 피드백 메시지에서 피드백 데이터 추출
        self.get_logger().info(f'Received feedback: {feedback}')  # 피드백 로그 출력

        msg = PackageStatusMsg()  # 패키지 상태 메시지 객체 생성
        msg.package_id = feedback.package_id  # 패키지 ID 설정
        msg.region = feedback.region  # 지역 설정
        msg.process = feedback.process  # 프로세스 설정

        self.packages_status.publish(msg)  # 패키지 상태 메시지 발행

    def plc_result_callback(self, future):  # 액션 결과 콜백 함수
        """서버에서 최종 결과를 받을 때 실행"""
        result = future.result().result  # 최종 결과 데이터 추출
        self.get_logger().info(f'Final result: plc_comp_action = {result.plc_result_action}')  # 결과 로그 출력
        self.is_cobot_move = False  # 코봇 이동 상태를 False로 설정
        self.cobot_action_complated_count += 1  # 코봇 액션 완료 횟수 증가
        if self.cobot_action_complated_count == TOTAL_PACKAGE:  # 완료 횟수가 총 패키지 수와 같다면
            self.pub_plc_complate()  # PLC 완료 메시지 발행
            self.pub_package_delete()  # 패키지 삭제 메시지 발행
            self.cobot_init_publisher.publish(Bool(data=True))  # 코봇 초기화 메시지 발행

    def pub_plc_complate(self):  # PLC 완료 메시지 발행 함수
        msg = Bool()  # Bool 메시지 객체 생성
        msg.data = True  # 데이터 True 설정
        self.is_agv_ready = False  # AGV 준비 상태 False로 설정
        self.get_logger().info(f'pub: {msg.data}')  # 발행 로그 출력

        self.plc_complate_publisher.publish(msg)  # PLC 완료 메시지 발행

    def pub_package_delete(self):  # 패키지 삭제 메시지 발행 함수
        msg = Bool()  # Bool 메시지 객체 생성
        msg.data = True  # 데이터 True 설정
        self.package_del_publisher.publish(msg)  # 패키지 삭제 메시지 발행

    # PLC에서 값 읽기 함수
    def read_register(self, address, count, slave_id, single_register=True):  # 레지스터 읽기 함수 정의
        try:  # 예외 처리 시작
            response = self.plc_client.read_holding_registers(address, count=count, slave=slave_id)  # PLC 레지스터 읽기 요청
            if response and not response.isError():  # 응답이 있고 에러가 없다면
                return response.registers[0] if single_register else response.registers  # 단일 레지스터면 첫 번째 값, 아니면 전체 값 반환
            else:  # 응답이 없거나 에러가 있다면
                print(f"PLC {slave_id} 슬레이브 {address}번 주소 읽기 실패!")  # 읽기 실패 로그 출력 (원본에서 주석 해제)
                return None  # None 반환
        except ModbusIOException:  # Modbus IO 예외 발생 시
            print(f"Modbus IO 오류 발생: 슬레이브 {slave_id}, 주소 {address}")  # IO 오류 로그 출력 (원본에서 주석 해제)
            return None  # None 반환
        except Exception as e:  # 기타 예외 발생 시
            print(f"read 예외 발생: {e}")  # 예외 로그 출력 (원본에서 주석 해제)
            return None  # None 반환

    # PLC에 값 쓰기 함수
    def write_register(self, address, value, slave_id):  # 레지스터 쓰기 함수 정의
        try:  # 예외 처리 시작
            response = self.plc_client.write_register(address, value, slave=slave_id)  # PLC 레지스터 쓰기 요청
            if not response or response.isError():  # 응답이 없거나 에러가 있다면
                print(f"PLC {slave_id}번 슬레이브 {address}번 주소에 값 {value} 쓰기 실패!")  # 쓰기 실패 로그 출력 (원본에서 주석 해제)
                return False  # False 반환
            print(f"PLC {slave_id}번 슬레이브 {address}번 주소에 값 {value} 쓰기 성공!")  # 쓰기 성공 로그 출력 (원본에서 주석 해제)
            return True  # True 반환
        except ModbusIOException:  # Modbus IO 예외 발생 시
            print(f"Modbus IO 오류 발생: 슬레이브 {slave_id}, 주소 {address}")  # IO 오류 로그 출력 (원본에서 주석 해제)
            return False  # False 반환
        except Exception as e:  # 기타 예외 발생 시
            print(f"write 예상치 못한 오류 발생: {e}")  # 예외 로그 출력 (원본에서 주석 해제)
            return False  # False 반환

    def plc_thread(self):  # PLC 스레드 함수
        while self.thread_running:  # 스레드가 실행 중일 때 반복
            manual = self.read_register(10, 1, SLAVE_ID_1)  # 10번 주소에서 수동 모드 값 읽기 (SLAVE_ID_1)
            auto = self.read_register(0, 1, SLAVE_ID_1)  # 0번 주소에서 자동 모드 값 읽기 (SLAVE_ID_1)
            stop = self.read_register(120, 1, SLAVE_ID_1)  # 120번 주소에서 정지 값 읽기 (SLAVE_ID_1)

            self.setting_mode()  # 모드 설정 함수 호출

            if stop == 1 and self.is_move is True:  # 정지 신호가 있고 이동 중이라면
                self.write_register(5, 5057, SLAVE_ID_2)  # 5번 주소에 5057 써서 인버터 정지 (SLAVE_ID_2)
                self.is_move = False  # 이동 상태 False로 설정
                self.Mode = None  # 모드 초기화

            if manual == 1:  # 수동 모드 값이 1이라면
                self.Mode = PlcMode.Manual  # 모드를 수동으로 설정
            elif auto == 1:  # 자동 모드 값이 1이라면
                self.Mode = PlcMode.Auto  # 모드를 자동으로 설정
            # manual mode---------------------------------------------------------------------------------------
            if self.Mode == PlcMode.Manual:  # 현재 모드가 수동이라면
                if self.is_move is False:  # 이동 중이 아니라면
                    # 30번(역방향)과 20번(정방향) 주소에서 값 읽기
                    backward = self.read_register(30, 1, SLAVE_ID_1)  # 30번 주소에서 역방향 값 읽기 (SLAVE_ID_1)
                    forward = self.read_register(20, 1, SLAVE_ID_1)  # 20번 주소에서 정방향 값 읽기 (SLAVE_ID_1)

                    # 역방향이면 5060 쓰기
                    if backward == 1:  # 역방향 신호가 1이라면
                        self.write_register(5, 5060, SLAVE_ID_2)  # 5번 주소에 5060 써서 역방향 이동 (SLAVE_ID_2)
                        self.is_move = True  # 이동 상태 True로 설정
                    # 정방향이면 5058 쓰기
                    elif forward == 1:  # 정방향 신호가 1이라면
                        self.write_register(5, 5058, SLAVE_ID_2)  # 5번 주소에 5058 써서 정방향 이동 (SLAVE_ID_2)
                        self.is_move = True  # 이동 상태 True로 설정

            # AutoMode---------------------------------------------------------------------------------------------
            elif self.Mode == PlcMode.Auto:  # 현재 모드가 자동이라면
                sensor = self.read_register(410, 1, SLAVE_ID_1)  # 410번 주소에서 센서 값 읽기 (SLAVE_ID_1)
                if sensor == 1 and self.is_move is True:  # 센서가 감지되고 이동 중이라면
                    self.write_register(5, 5057, SLAVE_ID_2)  # 5번 주소에 5057 써서 정지 (SLAVE_ID_2)
                    self.is_move = False  # 이동 상태 False로 설정
                    self.is_sensor = True  # 센서 상태 True로 설정

                if self.is_cobot_move == False and self.is_sensor == True:  # 코봇이 이동 중이 아니고 센서가 감지되었다면
                    #self.send_request()  # 액션 요청 전송 (주석 처리됨)
                    self.call_packageinfo_service()  # 패키지 정보 서비스 호출
                    self.is_sensor = False  # 센서 상태 False로 설정

                if self.is_agv_ready is True and self.is_move is False and sensor == 0:  # AGV 준비 완료, 이동 중이 아니고 센서 미감지 시
                    time.sleep(2)  # 2초 대기
                    self.write_register(5, 5058, SLAVE_ID_2)  # 5번 주소에 5058 써서 정방향 이동 (SLAVE_ID_2)
                    self.is_plc_ready_value = False  # PLC 준비 상태 False로 설정
                    self.is_move = True  # 이동 상태 True로 설정
                elif self.is_agv_ready is False and self.is_move is True:  # AGV 준비 미완료이고 이동 중이라면
                    self.write_register(5, 5057, SLAVE_ID_2)  # 5번 주소에 5057 써서 정지 (SLAVE_ID_2)
                    self.is_move = False  # 이동 상태 False로 설정

    def setting_mode(self):  # 모드 설정 및 데이터 모니터링 함수
        global prev_values, prev_inv_values, prev_popup_status, prev_send_addr5  # 전역 변수 선언
        # global plc_client  # 전역 plc_client 변수 (주석 처리됨)
        # 데이터 모니터링
        send_addr5 = self.read_register(120, 1, SLAVE_ID_1)  # 120번 주소에서 값 읽기 (SLAVE_ID_1)
        send_addr6 = self.read_register(121, 1, SLAVE_ID_1)  # 121번 주소에서 값 읽기 (SLAVE_ID_1)
        inv1_value = self.read_register(50, 3, SLAVE_ID_1, False)  # 50번 주소에서 3개 레지스터 값 읽기 (SLAVE_ID_1)

        if inv1_value is not None and isinstance(inv1_value, list) and len(inv1_value) >= 3:  # 읽은 값이 유효하고 리스트이며 길이가 3 이상이라면
            inv_value = inv1_value  # 읽은 값을 사용
        else:  # 그렇지 않다면
            inv_value = [0, 0, 0]  # 기본값 [0, 0, 0] 설정

        # 값 변경 감지 후 인버터 값 업데이트
        value_changed = False  # 값 변경 여부 변수 초기화
        if send_addr5 == 1 and send_addr6 == 1:  # 120번과 121번 주소 값이 모두 1이라면
            if prev_inv_values["freq"] != inv_value[0]:  # 이전 주파수 값과 현재 값이 다르다면
                if self.write_register(4, inv_value[0], SLAVE_ID_2):  # 4번 주소에 새 주파수 값 쓰기 (SLAVE_ID_2)
                    prev_inv_values["freq"] = inv_value[0]  # 이전 주파수 값 업데이트
                    value_changed = True  # 값 변경 플래그 True로 설정
                    print(f"Frequency value changed to {inv_value[0]}")  # 주파수 변경 로그 출력 (원본에서 주석 해제)

            if prev_inv_values["accel"] != inv_value[1]:  # 이전 가속 시간 값과 현재 값이 다르다면
                if self.write_register(6, inv_value[1], SLAVE_ID_2):  # 6번 주소에 새 가속 시간 값 쓰기 (SLAVE_ID_2)
                    prev_inv_values["accel"] = inv_value[1]  # 이전 가속 시간 값 업데이트
                    value_changed = True  # 값 변경 플래그 True로 설정
                    print(f"Accel value changed to {inv_value[1]}")  # 가속 시간 변경 로그 출력 (원본에서 주석 해제)

            if prev_inv_values["decel"] != inv_value[2]:  # 이전 감속 시간 값과 현재 값이 다르다면
                if self.write_register(7, inv_value[2], SLAVE_ID_2):  # 7번 주소에 새 감속 시간 값 쓰기 (SLAVE_ID_2)
                    prev_inv_values["decel"] = inv_value[2]  # 이전 감속 시간 값 업데이트
                    value_changed = True  # 값 변경 플래그 True로 설정
                    print(f"Decel value changed to {inv_value[2]}")  # 감속 시간 변경 로그 출력 (원본에서 주석 해제)

            if value_changed:  # 값이 변경되었다면
                print("Values updated:", prev_inv_values)  # 업데이트된 값 로그 출력 (원본에서 주석 해제)
        prev_popup_status = False  # 이전 팝업 상태 False로 초기화

        # 3번 주소에 1을 쓰고 초기화
        if value_changed and not prev_popup_status:  # 값이 변경되었고 이전 팝업 상태가 False라면
            success = self.write_register(3, 1, SLAVE_ID_1)  # 3번 주소에 1 쓰기 (SLAVE_ID_1)
            if success:  # 쓰기가 성공했다면
                time.sleep(0.5)  # 0.5초 대기
                self.write_register(3, 0, SLAVE_ID_1)  # 3번 주소에 0 쓰기 (SLAVE_ID_1)
                prev_popup_status = True  # 이전 팝업 상태 True로 설정
                print(f"prev_popup_status set to True: {prev_popup_status}")  # 상태 로그 출력 (원본에서 주석 해제)
            else:  # 쓰기가 실패했다면
                print("Failed to write to address 3")  # 실패 로그 출력 (원본에서 주석 해제)

        # 3번 주소에 값이 쓰였으면, prev_popup_status를 False로 리셋
        if prev_popup_status:  # 이전 팝업 상태가 True라면
            prev_popup_status = False  # 상태를 False로 리셋
            print(f"prev_popup_status reset to False: {prev_popup_status}")  # 리셋 로그 출력 (원본에서 주석 해제)

        # PLC에서 인버터 모니터링 값 읽기 (6번 ~ 9번 주소)
        inv_monitor = self.read_register(5, 5, SLAVE_ID_2, False) or [0, 0, 0, 0, 0]  # 5번 주소에서 5개 레지스터 값 읽기, 실패 시 기본값 설정 (SLAVE_ID_2)
        print(f"inv_monitor 값: {inv_monitor}")  # 모니터링 값 로그 출력 (원본에서 주석 해제)

        # 매핑된 주소 값 정의
        new_values = {
            200: inv_monitor[3],  # 인버터 현재 출력 (8번 주소)
            210: inv_monitor[4],  # 인버터 현재 주파수 (9번 주소)
            220: inv_monitor[1],  # 인버터 현재 가속 시간 (6번 주소)
            230: inv_monitor[2],  # 인버터 현재 감속 시간 (7번 주소)
            240: inv_monitor[0],  # 인버터 현재 진행 방향 (5번 주소)
        }

        # 변경이 발생한 경우에만 값 쓰기
        for addr, new_value in new_values.items():  # 각 주소와 새 값 순회
            if prev_values[addr] != new_value:  # 이전 값과 새 값이 다르다면
                success = self.write_register(addr, new_value, SLAVE_ID_1)  # 해당 주소에 새 값 쓰기 (SLAVE_ID_1)
                if success:  # 쓰기가 성공했다면
                    print(f"주소 {addr}에 {new_value} 쓰기 성공!")  # 쓰기 성공 로그 출력 (원본에서 주석 해제)
                    prev_values[addr] = new_value  # 이전 값 업데이트
                else:  # 쓰기가 실패했다면
                    print(f"주소 {addr}에 {new_value} 쓰기 실패!")  # 쓰기 실패 로그 출력 (원본에서 주석 해제)

    def destroy_node(self):  # 노드 종료 함수
        self.thread_running = False  # 스레드 실행 상태 False로 설정
        if self.thread.is_alive():  # 스레드가 실행 중이라면
            self.thread.join()  # 스레드 종료 대기
            self.write_register(5, 5057, SLAVE_ID_2)  # 5번 주소에 5057 써서 정지 (SLAVE_ID_2)
        super().destroy_node()  # 부모 클래스(Node)의 종료 함수 호출

# PLC 연결 함수
def connect_to_plc():  # PLC 연결 함수 정의
    global PORT, BAUDRATE, PARITY, STOPBITS, BYTESIZE, TIMEOUT  # 전역 변수 선언
    client = ModbusClient(  # Modbus 클라이언트 객체 생성
        framer='rtu',  # RTU 프레이머 설정
        port=PORT,  # 포트 설정
        baudrate=BAUDRATE,  # 보드레이트 설정
        parity=PARITY,  # 패리티 설정
        stopbits=STOPBITS,  # 스톱 비트 설정
        bytesize=BYTESIZE,  # 데이터 비트 수 설정
        timeout=TIMEOUT  # 타임아웃 설정
    )
    if client.connect():  # PLC 연결 시도
        print("PLC와 연결 성공!")  # 연결 성공 로그 출력
        return client  # 클라이언트 객체 반환
    else:  # 연결 실패 시
        print("PLC와 연결 실패!")  # 연결 실패 로그 출력
        return None  # None 반환

def main(args=None):  # 메인 함수 정의
    rclpy.init(args=args)  # ROS 2 초기화

    plc_client = connect_to_plc()  # PLC 연결 시도
    if plc_client is None:  # 연결 실패 시
        print("PLC 연결 실패!")  # 실패 로그 출력
        return  # 함수 종료

    plc_node = PlcNode(plc_client)  # PLC 노드 객체 생성

    try:  # 예외 처리 시작
        rclpy.spin(plc_node)  # 노드 실행 (무한 루프)
    except KeyboardInterrupt:  # 키보드 인터럽트 발생 시
        plc_node.get_logger().info("KeyboardInterrupt Stop.")  # 종료 로그 출력
    finally:  # 종료 시
        plc_node.destroy_node()  # 노드 종료 함수 호출

if __name__ == "__main__":  # 스크립트가 직접 실행될 때
    main()  # 메인 함수 호출
