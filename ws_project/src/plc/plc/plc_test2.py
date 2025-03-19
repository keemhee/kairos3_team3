import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException
from std_msgs.msg import Bool
import time
import threading
from std_msgs.msg import Int32
import re
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle 
from rclpy.action import ActionClient
from kairos_interfaces.action import Agvready
from kairos_interfaces.action import PlcCobot


# 글로벌 변수 설정
SLAVE_ID_1 = 1
SLAVE_ID_2 = 2
MAX_RETRIES = 2
PORT = '/dev/ttyUSB0'  # 예시: '/dev/ttyUSB0'
BAUDRATE = 9600
PARITY = 'N'
STOPBITS = 1
BYTESIZE = 8
TIMEOUT = 1
mutex = threading.Lock()


# PLC 연결 함수
def connect_to_plc(port, baudrate, parity, stopbits, bytesize, timeout):
    client = ModbusClient(
        framer='rtu',
        port=port,
        baudrate=baudrate,
        parity=parity,
        stopbits=stopbits,
        bytesize=bytesize,
        timeout=timeout
    )
    if client.connect():
        print("PLC와 연결 성공!")
        return client
    else:
        print("PLC와 연결 실패!")
        return None
    

# PLC에서 값 읽기
def read_register(client, address, count, slave_id, single_register=False):
    try:
        response = client.read_holding_registers(address, count = count, slave=slave_id)
        if response and not response.isError():
            return response.registers[0] if single_register else response.registers
        else:
            print(f"PLC {slave_id} 슬레이브 {address}번 주소 읽기 실패!")
            return None
    except ModbusIOException:
        print(f"Modbus IO 오류 발생: 슬레이브 {slave_id}, 주소 {address}")
        return None
    except Exception as e:
        print(f"read 예외 발생: {e}")
        return None

# PLC에 값 쓰기
def write_register(client, address, value, slave_id):
    try:
        response = client.write_register(address, value, slave=slave_id)
        if not response or response.isError():
            print(f"PLC {slave_id}번 슬레이브 {address}번 주소에 값 {value} 쓰기 실패!")
            return False
        print(f"PLC {slave_id}번 슬레이브 {address}번 주소에 값 {value} 쓰기 성공!")
        return True
    except ModbusIOException:
        print(f"Modbus IO 오류 발생: 슬레이브 {slave_id}, 주소 {address}")
        return False
    except Exception as e:
        print(f"write 예상치 못한 오류 발생: {e}")
        return False
    
# 쓰기 함수 수정
def write_register_with_retry(plc_client, register, value, slave_id, retries=MAX_RETRIES):
    for attempt in range(retries):
        if write_register(plc_client, register, value, slave_id):
            return True
        print(f"쓰기 실패, {attempt + 1}번째 시도 중...")
        time.sleep(0.5)
    print(f"쓰기 실패, 최대 재시도 횟수({retries}) 초과!")
    return False  

plc_client = connect_to_plc(PORT, BAUDRATE, PARITY, STOPBITS, BYTESIZE, TIMEOUT)

#initiate setting
write_register(plc_client,120,0,SLAVE_ID_1)
write_register(plc_client,10,1,SLAVE_ID_1)
write_register(plc_client,4,800,SLAVE_ID_2)
write_register(plc_client,6,0,SLAVE_ID_2)
write_register(plc_client,7,0,SLAVE_ID_2)
write_register(plc_client,130,800,SLAVE_ID_2)


# stop_event와 lock 생성
stop_event = threading.Event()
lock = threading.Lock()

# 이전 값 저장용 딕셔너리 (초기값은 None)
prev_values = {200: None, 210: None, 220: None, 230: None, 240: None}   
prev_inv_values = {"freq": 0, "accel": 0, "decel": 0}
prev_popup_status = False
prev_send_addr5 = None



class Plc(Node):
    def __init__(self):
        super().__init__("Plc_node")
        global plc_client
        self.get_logger().info('Plc_node has started')
        # 오토 노드 실행 스레드 추가

        
        self.is_agv_ready = False  # 여기에 is_agv_ready 속성을 추가합니다.
        self.sensor_detech = False
        # self._future = None
    
        self.action_server_ = ActionServer(self, Agvready, 'agv_ready', self.result_callback)
        # self.action_client_ = ActionClient(self, PlcCobot,"plc_cobot")
        # self.future = PlcCobot.request()


        
        self.get_logger().info("Server has been started")

        if plc_client is None:
            self.get_logger().error("PLC 연결 실패!")
            rclpy.shutdown()
            return
        

        auto_thread = threading.Thread(target=self.handle_automation_conditions, args=(plc_client,))
        auto_thread.start()

        # 매뉴얼 노드 실행 스레드 추가
        manual_thread = threading.Thread(target=self.handle_manual_mode, args=(plc_client,))
        manual_thread.start()


    # #Action Client def
    # def send_request(self, is_plc_ready_value):        
    #     req_msg=PlcCobot.Goal
    #     req_msg.is_plc_ready = is_plc_ready_value
    #     self.get_logger().info(f'Sending goal: is_plc_ready = {is_plc_ready_value}')
    #     self.action_client_.wait_for_server()
    #     self._future = self.action_client_.send_goal_async(req_msg, feedback_callback=self.feedback_callback)
    #     self._future.add_done_callback(self.goal_response_callback)

    # def goal_response_callback(self, _future):
    #     """서버에서 목표 수락 여부 확인"""
    #     goal_handle = _future.result()

    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected')
    #         return

    #     self.get_logger().info('Goal accepted')

    #     # 결과 요청 및 콜백 함수 등록
    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.plc_result_callback)

    # def feedback_callback(self, feedback_msg):
    #     """서버에서 실시간 피드백을 받을 때 실행"""
    #     feedback = feedback_msg.feedback
    #     self.get_logger().info(f'Received feedback: plc_comp_action = {feedback.plc_feedback_action}')

    # def plc_result_callback(self, future):
    #     """서버에서 최종 결과를 받을 때 실행"""
    #     result = future.result().result
    #     self.get_logger().info(f'Final result: plc_comp_action = {result.plc_result_action}')

    #     # 노드 종료
    #     rclpy.shutdown()


    #Agv Action def
    def result_callback(self, goal_handle: ServerGoalHandle):
        global plc_client
        # result 변수 초기화
        result = Agvready.Result()

        self.get_logger().info(f"액션 요청 도착: {goal_handle.request}")

        # 요청이 올바르게 수신되었는지 확인
        if read_register(plc_client,10,1,SLAVE_ID_1,single_register=True):
            goal_handle.abort()
            return #Agvready.Result(complate_action=False)
        
        else:
            goal_handle.succeed()
            self.is_agv_ready = goal_handle.request.is_agv_ready
            self.get_logger().info(f"Received request: is_agv_ready = {self.is_agv_ready}")
            #auto_result = self.handle_automation_conditions()

        while True:
            if self.sensor_detech is True:
                result.complate_action = True  # 액션 완료 응답 
                break
                  
        return result
            
    # 자동화 조건 처리 함수
    def handle_automation_conditions(self, plc_client):
        is_move = False
        while True:
            step_1 = read_register(plc_client, 400, 1, SLAVE_ID_1, single_register=True)
            step_2 = read_register(plc_client, 411, 1, SLAVE_ID_1, single_register=True)
            
            print(f"step_1: {step_1}, step_2: {step_2}, is_agv_ready: {self.is_agv_ready}")

            if step_2 == 1 and is_move is True:
                write_register(plc_client, 5, 5057, SLAVE_ID_2)  # 정지
                print("PLC 5번 주소에 5057 값을 썼습니다. 정지 명령.")  # 디버깅 메시지 추가
                self.send_request()
                is_move = False
                self.sensor_detech = True
                

            elif step_1 == 1 and self.is_agv_ready == 1 and is_move is False:
                write_register(plc_client,0,1,SLAVE_ID_1)
                write_register(plc_client, 5, 5058, SLAVE_ID_2)  # 정방향
                is_move =True
                print("PLC 5번 주소에 5058 값을 썼습니다. 정방향 명령.")  # 디버깅 메시지 추가
                self.sensor_detech = False
    
    
    def handle_manual_mode(self, plc_client):
        Once = False
        while True:
            # 10번 주소에서 값 읽기
            send_addr4 = read_register(plc_client,410 ,1, SLAVE_ID_1,single_register=True)
            send_addr1 = read_register(plc_client, 10, 1, SLAVE_ID_1, single_register=True)

            print("manual")
            if send_addr1 == 1:
                # 30번(역방향)과 20번(정방향) 주소에서 값 읽기
                send_addr2 = read_register(plc_client, 30, 1, SLAVE_ID_1, single_register=True)  # 역방향
                send_addr3 = read_register(plc_client, 20, 1, SLAVE_ID_1, single_register=True)  # 정방향
                
                # 역방향이면 5060 쓰기
                if send_addr2 == 1 and Once is False:
                    write_register(plc_client, 5, 5060, SLAVE_ID_2)
                    Once = True
                    print("once true")
                # 정방향이면 5058 쓰기
                elif send_addr3 == 1 and Once is False:
                    write_register(plc_client, 5, 5058, SLAVE_ID_2)
                    Once =True
                    print("once true")
            if send_addr4 == 1 and Once is True:
                write_register(plc_client,5,5057,SLAVE_ID_2)
                Once = False
                print("once false")

            self.setting_mode()

    def setting_mode(self):
        global prev_values, prev_inv_values, prev_popup_status, prev_send_addr5
        # global plc_client
        # 데이터 모니터링 (기존 run_data_monitoring 내용)
        send_addr5 = read_register(plc_client, 410, 1, SLAVE_ID_1, single_register=True)
        send_addr6 = read_register(plc_client, 121, 1, SLAVE_ID_1, single_register=True)
        inv1_value = read_register(plc_client, 50, 3, SLAVE_ID_1)
        Once = False
        # print("test")
        if inv1_value is not None and isinstance(inv1_value, list) and len(inv1_value) >= 3:
            inv_value = inv1_value
        else:
            inv_value = [0, 0, 0]

        # 값 변경 감지 후 인버터 값 업데이트
        value_changed = False
        if send_addr5 == 1 and send_addr6 == 1 :
            if prev_inv_values["freq"] != inv_value[0]:
                if write_register(plc_client, 4, inv_value[0], SLAVE_ID_2):
                    prev_inv_values["freq"] = inv_value[0]
                    value_changed = True
                    print(f"Frequency value changed to {inv_value[0]}")

            if prev_inv_values["accel"] != inv_value[1]:
                if write_register(plc_client, 6, inv_value[1], SLAVE_ID_2):
                    prev_inv_values["accel"] = inv_value[1]
                    value_changed = True
                    print(f"Accel value changed to {inv_value[1]}")

            if prev_inv_values["decel"] != inv_value[2]:
                if write_register(plc_client, 7, inv_value[2], SLAVE_ID_2):
                    prev_inv_values["decel"] = inv_value[2]
                    value_changed = True
                    print(f"Decel value changed to {inv_value[2]}")

            if value_changed:
                print("Values updated:", prev_inv_values)  # 디버깅용        
        prev_popup_status = False
        # 3번 주소에 1을 쓰고 초기화
        if value_changed and not prev_popup_status:  # value_changed가 True일 때만 실행
            success = write_register_with_retry(plc_client, 3, 1, SLAVE_ID_1)
            if success:
                time.sleep(0.5)
                write_register_with_retry(plc_client, 3, 0, SLAVE_ID_1)
                prev_popup_status = True
                print(f"prev_popup_status set to True: {prev_popup_status}")  # 디버깅용
                print(f"{prev_inv_values}")
            else:
                print("Failed to write to address 3")

        # 3번 주소에 값이 쓰였으면, prev_popup_status를 False로 리셋
        if prev_popup_status:
            prev_popup_status = False  # 한 번 쓰고 나면 다시 False로 리셋
            print(f"prev_popup_status reset to False: {prev_popup_status}")  # 디버깅용

        # PLC에서 인버터 모니터링 값 읽기 (6번 ~ 9번 주소)
        inv_monitor = read_register(plc_client, 5, 5, SLAVE_ID_2) or [0, 0, 0, 0, 0]

        # 매핑된 주소 값
        new_values = {
            200: inv_monitor[3],  # 인버터 현재출력 (8번 주소)
            210: inv_monitor[4],  # 인버터 현재주파수 (9번 주소)
            220: inv_monitor[1],  # 인버터 현재가속시간 (6번 주소)
            230: inv_monitor[2],  # 인버터 현재감속시간 (7번 주소)
            240: inv_monitor[0],  # 인버터 현재진행방향 (5번 주소) 
        }

        # 변경이 발생한 경우에만 값 쓰기
        for addr, new_value in new_values.items():
            if prev_values[addr] != new_value:  # 값이 다를 경우에만 쓰기 실행
                write_register(plc_client, addr, new_value, SLAVE_ID_1)
                prev_values[addr] = new_value  # 이전 값 업데이트


def main(args=None):
    rclpy.init(args=args)

    Plc_node = Plc()

    try:
        # 필요한 경우 추가적인 로직을 여기에 작성
        rclpy.spin(Plc_node)

    except KeyboardInterrupt:
        pass
    finally:
        Plc_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()