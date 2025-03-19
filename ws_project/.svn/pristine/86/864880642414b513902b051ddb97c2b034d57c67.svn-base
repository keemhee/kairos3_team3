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


# 글로벌 변수 설정
SLAVE_ID_1 = 1
SLAVE_ID_2 = 2
MAX_RETRIES = 2
prev_inv_values = {"freq": 0, "accel": 0, "decel": 0}
prev_popup_status = False
prev_send_addr5 = None

PORT = '/dev/ttyUSB0'  # 예시: '/dev/ttyUSB0'
BAUDRATE = 9600
PARITY = 'N'
STOPBITS = 1
BYTESIZE = 8
TIMEOUT = 1

# stop_event와 lock 생성
stop_event = threading.Event()
lock = threading.Lock()


class PlcPub(Node):
    def __init__(self):
        super().__init__("plc_pub")
        self.get_logger().info('plc_pub has started')
        self.is_agv_ready = False  # 여기에 is_agv_ready 속성을 추가합니다.
        self.plc_client = self.connect_to_plc(PORT, BAUDRATE, PARITY, STOPBITS, BYTESIZE, TIMEOUT)
        self.write_register(self.plc_client,10,1,SLAVE_ID_1)
        self.write_register(self.plc_client,4,800,SLAVE_ID_2)
        self.write_register(self.plc_client,6,0,SLAVE_ID_2)
        self.write_register(self.plc_client,7,0,SLAVE_ID_2)

        # self.step_2_pub = self.create_publisher(Int32, "step_2_signal", 10)
        # self.subscription = self.create_subscription(String, "/QR_data", self.qr_data_callback, 10)
        
        
        self.action_server_ = ActionServer(self, Agvready, 'agv_ready', self.result_callback)
        self.get_logger().info("Server has been started")

        
        # 이전 값 저장용 딕셔너리 (초기값은 None)
        self.prev_values = {200: None, 210: None, 220: None, 230: None, 240: None}    

        # QR 코드에서 추출한 Region 값 저장
        self.region_data = None 


        if self.plc_client is None:
            self.get_logger().error("PLC 연결 실패!")
            rclpy.shutdown()
            return

        thread1 = threading.Thread(target=self.run_monitoring_and_control,args=(self.plc_client, stop_event, lock),daemon=True)  # 메인 스레드 종료 시 스레드도 종료))
        thread1.start()



    # def qr_data_callback(self, msg):
    #     qr_text = msg.data
    #     match = re.search(r"Region:\s*([\w가-힣]+)", qr_text)  # 정규표현식으로 'Region' 값 추출
    #     if match:
    #         self.region_data = match.group(1)
    #         self.get_logger().info(f"Region 값 업데이트: {self.region_data}")
    #         # Region 값이 '경기'일 때, PLC에 값 쓰기
    #         if self.region_data == "경기":
    #             plc_client = self.connect_to_plc(PORT, BAUDRATE, PARITY, STOPBITS, BYTESIZE, TIMEOUT)  # PLC 연결
    #             if plc_client:
    #                 success = self.write_register(plc_client, 234, 5, SLAVE_ID_1)  # 예시: 5번 주소에 1을 쓰기
    #                 self.write_register(plc_client,123,1,SLAVE_ID_1)
    #                 if success:
    #                     self.get_logger().info("PLC에 값 5을 성공적으로 썼습니다.")
    #                 else:
    #                     self.get_logger().error("PLC에 값 5을 쓰는 데 실패했습니다.")

    #         # Region 값이 '부산'일 때, PLC에 값 쓰기
    #         elif self.region_data == "부산":
    #             plc_client = self.connect_to_plc(PORT, BAUDRATE, PARITY, STOPBITS, BYTESIZE, TIMEOUT)  # PLC 연결
    #             if plc_client:
    #                 success = self.write_register(plc_client, 234, 7, SLAVE_ID_1)  # 예시: 5번 주소에 1을 쓰기
    #                 self.write_register(plc_client,123,1,SLAVE_ID_1)
    #                 if success:
    #                     self.get_logger().info("PLC에 값 7을 성공적으로 썼습니다.")
    #                 else:
    #                     self.get_logger().error("PLC에 값 7을 쓰는 데 실패했습니다.")


    #         # Region 값이 '서울'일 때, PLC에 값 쓰기
    #         elif self.region_data == "서울":
    #             plc_client = self.connect_to_plc(PORT, BAUDRATE, PARITY, STOPBITS, BYTESIZE, TIMEOUT)  # PLC 연결
    #             if plc_client:
    #                 success = self.write_register(plc_client, 234, 3, SLAVE_ID_1)  # 예시: 5번 주소에 1을 쓰기
    #                 self.write_register(plc_client,123,1,SLAVE_ID_1)
    #                 if success:
    #                     self.get_logger().info("PLC에 값 3을 성공적으로 썼습니다.")
   #                 else:
    #                     self.get_logger().error("PLC에 값 3을 쓰는 데 실패했습니다.")

    

    # def result_callback(self, future):
    #     """Action 결과를 비동기적으로 받아오는 콜백"""
    #     result = future.result()  # 액션 실행 결과를 가져옵니다.
    #     success = result.success  # 액션 결과에서 success 속성 추출
    #     self.get_logger().info(f"Action result: {success}")

    #     if success:
    #         self.get_logger().info("Action completed successfully!")
    #     else:
    #         self.get_logger().error("Action failed!")


    # def result_callback(self, goal_handle):
    #     is_agv_ready = getattr(goal_handle.request, "is_agv_ready", False)
    #     goal_handle.succeed()
        
    #     # 요청 값 출력
    #     self.get_logger().info(f"Received request: is_agv_ready = {is_agv_ready}")

    #     # 결과 값은 설정하지 않으므로 단순히 반환
    #     result = Agvready.Result()
    #     result.complate_action = True  # 예시로 '완료' 값을 True로 설정
    #     return result

    
    def result_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"액션 요청 도착: {goal_handle.request}")

        # 요청이 올바르게 수신되었는지 확인
        if not hasattr(goal_handle.request, "is_agv_ready"):
            self.get_logger().error("is_agv_ready 속성이 없습니다!")
            goal_handle.abort()
            return #Agvready.Result(complate_action=False)

        self.is_agv_ready = goal_handle.request.is_agv_ready
        self.get_logger().info(f"Received request: is_agv_ready = {self.is_agv_ready}")

        goal_handle.succeed()
        
        step_3 = self.read_register(self.plc_client, 410, 1, SLAVE_ID_1, single_register=True)
        if step_3 == 1:
            result = Agvready.Result()       
            result.complate_action = True  # 액션 완료 응답
            
        return result
            
    # PLC 연결 함수
    def connect_to_plc(self, port, baudrate, parity, stopbits, bytesize, timeout):
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
    def read_register(self, client, address, count, slave_id, single_register=False):
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
    def write_register(self, client, address, value, slave_id):
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

    def handle_manual_mode(self, plc_client):
        # 10번 주소에서 값 읽기
        send_addr1 = self.read_register(plc_client, 10, 1, SLAVE_ID_1, single_register=True)
        if send_addr1 == 1:
            # 30번(역방향)과 20번(정방향) 주소에서 값 읽기
            send_addr2 = self.read_register(plc_client, 30, 1, SLAVE_ID_1, single_register=True)  # 역방향
            send_addr3 = self.read_register(plc_client, 20, 1, SLAVE_ID_1, single_register=True)  # 정방향
            
            # 역방향이면 5060 쓰기
            if send_addr2 == 1:
                self.write_register(plc_client, 5, 5060, SLAVE_ID_2)
            # 정방향이면 5058 쓰기
            elif send_addr3 == 1:
                self.write_register(plc_client, 5, 5058, SLAVE_ID_2)
        

    # 자동화 조건 처리 함수
    def handle_automation_conditions(self, plc_client):
        step_1 = self.read_register(plc_client, 400, 1, SLAVE_ID_1, single_register=True) #
        step_2 = self.read_register(plc_client, 410, 1, SLAVE_ID_1, single_register=True)
        
        print(f"step_1: {step_1}, step_2: {step_2}, is_agv_ready: {self.is_agv_ready}")

        if step_2 == 1:
            self.write_register(plc_client, 5, 5057, SLAVE_ID_2)  # 정지
            print("PLC 5번 주소에 5057 값을 썼습니다. 정지 명령.")  # 디버깅 메시지 추가
            time.sleep(1)  # CPU 부하 방지

        elif self.is_agv_ready==1:
            self.write_register(plc_client,0,1,SLAVE_ID_1) 
            self.write_register(plc_client, 5, 5058, SLAVE_ID_2)  # 정방향
            print("PLC 5번 주소에 5058 값을 썼습니다. 정방향 명령.")  # 디버깅 메시지 추가

            self.is_agv_ready = 0

        elif step_2 ==1:
            self.write_register(plc_client, 5, 5058, SLAVE_ID_2)  # 정방향   
            print("PLC 5번 주소에 5058 값을 썼습니다. 정방향 명령.")  # 디버깅 메시지 추가 
            

    # 쓰기 함수 수정
    def write_register_with_retry(self, plc_client, register, value, slave_id, retries=MAX_RETRIES):
        for attempt in range(retries):
            if self.write_register(plc_client, register, value, slave_id):
                return True
            print(f"쓰기 실패, {attempt + 1}번째 시도 중...")
            time.sleep(0.5)
        print(f"쓰기 실패, 최대 재시도 횟수({retries}) 초과!")
        return False  

    def run_monitoring_and_control(self, plc_client, stop_event, lock):
        global prev_inv_values, prev_popup_status        

        while not stop_event.is_set():
            with lock:  # 한 번에 하나의 Modbus 요청만 처리
                # 자동화 조건 처리
                self.handle_automation_conditions(plc_client)

                # 데이터 모니터링 (기존 run_data_monitoring 내용)
                send_addr5 = self.read_register(plc_client, 410, 1, SLAVE_ID_1, single_register=True)
                send_addr6 = self.read_register(plc_client, 121, 1, SLAVE_ID_1, single_register=True)
                inv1_value = self.read_register(plc_client, 50, 3, SLAVE_ID_1)

                if inv1_value is not None and isinstance(inv1_value, list) and len(inv1_value) >= 3:
                    inv_value = inv1_value
                else:
                    inv_value = [0, 0, 0]

                # 값 변경 감지 후 인버터 값 업데이트
                value_changed = False
                if send_addr5 == 1 and send_addr6 == 1:
                    if prev_inv_values["freq"] != inv_value[0]:
                        if self.write_register(plc_client, 4, inv_value[0], SLAVE_ID_2):
                            prev_inv_values["freq"] = inv_value[0]
                            value_changed = True
                            print(f"Frequency value changed to {inv_value[0]}")

                    if prev_inv_values["accel"] != inv_value[1]:
                        if self.write_register(plc_client, 6, inv_value[1], SLAVE_ID_2):
                            prev_inv_values["accel"] = inv_value[1]
                            value_changed = True
                            print(f"Accel value changed to {inv_value[1]}")

                    if prev_inv_values["decel"] != inv_value[2]:
                        if self.write_register(plc_client, 7, inv_value[2], SLAVE_ID_2):
                            prev_inv_values["decel"] = inv_value[2]
                            value_changed = True
                            print(f"Decel value changed to {inv_value[2]}")

                    if value_changed:
                        print("Values updated:", prev_inv_values)  # 디버깅용        

                # 3번 주소에 1을 쓰고 초기화
                if value_changed and not prev_popup_status:  # value_changed가 True일 때만 실행
                    success = self.write_register_with_retry(plc_client, 3, 1, SLAVE_ID_1)
                    if success:
                        time.sleep(0.5)
                        self.write_register_with_retry(plc_client, 3, 0, SLAVE_ID_1)
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
                inv_monitor = self.read_register(plc_client, 5, 5, SLAVE_ID_2) or [0, 0, 0, 0, 0]

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
                    if self.prev_values[addr] != new_value:  # 값이 다를 경우에만 쓰기 실행
                        self.write_register(plc_client, addr, new_value, SLAVE_ID_1)
                        self.prev_values[addr] = new_value  # 이전 값 업데이트

                self.handle_manual_mode(plc_client)

            time.sleep(1)  # CPU 부하 방지

def main(args=None):
    rclpy.init(args=args)
    node = PlcPub()
    
    # PLC 모니터링을 동기적으로 처리
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
