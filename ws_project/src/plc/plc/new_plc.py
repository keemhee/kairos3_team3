import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException
import time
import threading
from enum import Enum
from kairos_interfaces.action import PlcCobot
from kairos_interfaces.srv import PackageInfo
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from kairos_interfaces.msg import PackageStatusMsg

# 글로벌 변수 설정
SLAVE_ID_1 = 1
SLAVE_ID_2 = 2
MAX_RETRIES = 2
prev_values = {200: None, 210: None, 220: None, 230: None, 240: None}
prev_values_init = {130: None, 140: None,150:None}  
prev_inv_values = {"freq": 0, "accel": 0, "decel": 0}
prev_popup_status = False
prev_send_addr5 = None

PORT = '/dev/ttyUSB0'  # 예시: '/dev/ttyUSB0'
BAUDRATE = 9600
PARITY = 'N'
STOPBITS = 1
BYTESIZE = 8
TIMEOUT = 1

INIT_FRQ = 600
INIT_ACCEL = 10
INIT_DECEL = 10

TOTAL_PACKAGE = 10

class PlcMode(Enum):
    Manual = 1
    Auto = 2

class PlcNode(Node):
    def __init__(self, plc_client):
        super().__init__("Plc_Node")
        self.get_logger().info("Plc Node has been Started...")

        self.agv_ready_subscription = self.create_subscription(Bool, "agv_ready", self.agv_ready_callback, 10 )
        self.plc_complate_publisher = self.create_publisher(Bool, "plc_complate", 10)
        self.package_info_service_client = self.create_client(PackageInfo, "package_info")
        self.plc_client_ = ActionClient(self, PlcCobot, 'pick_place_action')
        self.package_del_publisher = self.create_publisher(Bool, 'package_delete', 10)
        self.cobot_init_publisher = self.create_publisher(Bool, 'cobot_init', 10)
        self.packages_status = self.create_publisher(PackageStatusMsg, 'packages_status', 10)

        self.is_cobot_move =False
        self.cobot_action_complated_count = 0
        self.is_plc_ready_value = False

        self.plc_client = plc_client
        self.Mode = PlcMode.Manual
        self.is_move = False
        self.thread_running = True

        self.is_sensor = False
        self.is_agv_ready = False
        self.initiate_setting()

        self.thread = threading.Thread(target=self.plc_thread, daemon=True)
        self.thread.start()

            
    def initiate_setting(self):
        self.write_register(120,1,SLAVE_ID_1) #stop
        self.write_register(5,5057,SLAVE_ID_2) #stop_inv
        self.write_register(120,0,SLAVE_ID_1) #stop_init
        self.write_register(10,1,SLAVE_ID_1) #manual
        self.write_register(4,INIT_FRQ,SLAVE_ID_2) #frq
        self.write_register(6,INIT_ACCEL,SLAVE_ID_2) #accel
        self.write_register(7,INIT_DECEL,SLAVE_ID_2) #decel
        
        time.sleep(1)

        prev_inv_values["freq"] = INIT_FRQ
        prev_inv_values["accel"] = INIT_ACCEL
        prev_inv_values["decel"] = INIT_DECEL

        print(f"Read values - Frq: {INIT_FRQ}, Accel: {INIT_ACCEL}, Decel: {INIT_DECEL}")

        # 매핑된 주소 값
        new_values_init = {
            130: INIT_FRQ,  # 인버터 현재주파수
            140: INIT_ACCEL,  # 인버터 현재가속시간
            150: INIT_DECEL,  # 인버터 현재감속시간
        }

        # 변경이 발생한 경우에만 값 쓰기
        for index, (addr, new_value) in enumerate(new_values_init.items()):
            if prev_values_init[addr] != new_value:  # 값이 다를 경우에만 쓰기 실행
                self.write_register(addr, new_value, SLAVE_ID_1)
                self.write_register(50 + index, new_value, SLAVE_ID_1)
                prev_values_init[addr] = new_value  # 이전 값 업데이트

    def agv_ready_callback(self, msg):
        self.get_logger().info(f'sub: {msg.data}')
        #Manual Mode return
        if self.Mode == PlcMode.Manual:
            return
        
        if msg.data is True:
            self.is_agv_ready = True
            self.cobot_action_complated_count = 0
          

    def call_packageinfo_service(self):
        self.is_cobot_move = True
        request = PackageInfo.Request()
        self.get_logger().info(f'Send Service')
        future = self.package_info_service_client.call_async(request)
        future.add_done_callback(self.packageinfo_response_callback)

    def packageinfo_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response}')

            action_request = PlcCobot.Goal()
            action_request.is_plc_ready = True
            action_request.package_id = response.package_id
            action_request.region = response.region
            action_request.width = response.width
            action_request.height = response.height
            action_request.depth = response.depth
            action_request.process = response.process
            action_request.x = response.x
            action_request.y = response.y
            action_request.theta = response.theta

            #Log send
            logdata = 0
            if response.region == "서울":
                logdata = 3
            elif response.region == "경기":
                logdata = 5
            elif response.region == "부산":
                logdata = 7

            self.write_register(234, logdata, SLAVE_ID_1)
            self.write_register(123, 1, SLAVE_ID_1)
            time.sleep(0.1)
            self.write_register(123, 0, SLAVE_ID_1)
            self.send_request(action_request)

        except Exception as e:
            self.get_logger().warn(f'{e}')
    
    #Action Client def
    def send_request(self, action_reauest):        

        self.get_logger().info(f'Sending goal')
        self.plc_client_.wait_for_server()
        #self.get_logger().info(f'Action server is available')

        future = self.plc_client_.send_goal_async(action_reauest, feedback_callback=self.plc_feedback_callback)
        #self.get_logger().info(f'Goal sent, waiting for response...')
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """서버에서 목표 수락 여부 확인"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # 결과 요청 및 콜백 함수 등록
        _get_result_future = goal_handle.get_result_async()
        _get_result_future.add_done_callback(self.plc_result_callback)

    def plc_feedback_callback(self, feedback_msg):
        """서버에서 실시간 피드백을 받을 때 실행"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

        msg = PackageStatusMsg()
        msg.package_id = feedback.package_id
        msg.region = feedback.region
        msg.process = feedback.process

        self.packages_status.publish(msg)

    def plc_result_callback(self, future):
        """서버에서 최종 결과를 받을 때 실행"""
        result = future.result().result
        self.get_logger().info(f'Final result: plc_comp_action = {result.plc_result_action}')
        self.is_cobot_move = False
        self.cobot_action_complated_count += 1
        if self.cobot_action_complated_count == TOTAL_PACKAGE:
            self.pub_plc_complate()
            self.pub_package_delete()
            self.cobot_init_publisher.publish(Bool(data=True))

    def pub_plc_complate(self):
        msg = Bool()
        msg.data = True
        self.is_agv_ready = False
        self.get_logger().info(f'pub: {msg.data}')

        self.plc_complate_publisher.publish(msg)

    def pub_package_delete(self):
        msg = Bool()
        msg.data = True
        self.package_del_publisher.publish(msg)

    # PLC에서 값 읽기
    def read_register(self, address, count, slave_id, single_register=True):
        try:
            response = self.plc_client.read_holding_registers(address, count = count, slave=slave_id)
            if response and not response.isError():
                return response.registers[0] if single_register else response.registers
            else:
                #print(f"PLC {slave_id} 슬레이브 {address}번 주소 읽기 실패!")
                return None
        except ModbusIOException:
            #print(f"Modbus IO 오류 발생: 슬레이브 {slave_id}, 주소 {address}")
            return None
        except Exception as e:
            #print(f"read 예외 발생: {e}")
            return None


    # PLC에 값 쓰기
    def write_register(self, address, value, slave_id):
        try:
            response = self.plc_client.write_register(address, value, slave=slave_id)
            if not response or response.isError():
                #print(f"PLC {slave_id}번 슬레이브 {address}번 주소에 값 {value} 쓰기 실패!")
                return False
            #print(f"PLC {slave_id}번 슬레이브 {address}번 주소에 값 {value} 쓰기 성공!")
            return True
        except ModbusIOException:
            #print(f"Modbus IO 오류 발생: 슬레이브 {slave_id}, 주소 {address}")
            return False
        except Exception as e:
            #print(f"write 예상치 못한 오류 발생: {e}")
            return False

    def plc_thread(self):
        while self.thread_running:
            manual = self.read_register( 10, 1, SLAVE_ID_1)
            auto = self.read_register( 0, 1, SLAVE_ID_1)
            stop = self.read_register(120 ,1, SLAVE_ID_1)
        
            self.setting_mode()
        
            if stop == 1 and self.is_move is True:
                self.write_register(5,5057,SLAVE_ID_2)
                self.is_move = False
                self.Mode = None

            if manual == 1:
                self.Mode = PlcMode.Manual
            elif auto == 1:
                self.Mode = PlcMode.Auto
            #manual mode---------------------------------------------------------------------------------------
            if self.Mode == PlcMode.Manual:
                if self.is_move is False:
                    # 30번(역방향)과 20번(정방향) 주소에서 값 읽기
                    backward = self.read_register(30, 1, SLAVE_ID_1)  # 역방향
                    forward = self.read_register(20, 1, SLAVE_ID_1)  # 정방향

                    # 역방향이면 5060 쓰기
                    if backward == 1:
                        self.write_register( 5, 5060, SLAVE_ID_2)
                        self.is_move = True
                    # 정방향이면 5058 쓰기
                    elif forward == 1:
                        self.write_register(5, 5058, SLAVE_ID_2)
                        self.is_move = True
                        
            #AutoMode---------------------------------------------------------------------------------------------
            elif self.Mode == PlcMode.Auto:
                sensor = self.read_register(410 ,1, SLAVE_ID_1)
                if sensor == 1 and self.is_move is True:
                    self.write_register(5, 5057, SLAVE_ID_2)  # 정지
                    self.is_move = False
                    self.is_sensor = True
                    
                if self.is_cobot_move == False and self.is_sensor == True:
                    #self.send_request()
                    self.call_packageinfo_service()
                    self.is_sensor = False

                if self.is_agv_ready is True and self.is_move is False and sensor == 0:
                    time.sleep(2)
                    self.write_register(5, 5058, SLAVE_ID_2)  # 정방향
                    self.is_plc_ready_value = False
                    self.is_move = True
                elif self.is_agv_ready is False and self.is_move is True:
                    self.write_register(5, 5057, SLAVE_ID_2)
                    self.is_move = False

    
    def setting_mode(self):
        global prev_values, prev_inv_values, prev_popup_status, prev_send_addr5
        # global plc_client
        # 데이터 모니터링 (기존 run_data_monitoring 내용)
        send_addr5 = self.read_register(120, 1, SLAVE_ID_1)
        send_addr6 = self.read_register(121, 1, SLAVE_ID_1)
        inv1_value = self.read_register(50, 3, SLAVE_ID_1, False)


        if inv1_value is not None and isinstance(inv1_value, list) and len(inv1_value) >= 3:
            inv_value = inv1_value
        else:
            inv_value = [0, 0, 0]

        # 값 변경 감지 후 인버터 값 업데이트
        value_changed = False
        if send_addr5 == 1 and send_addr6 == 1 :
            if prev_inv_values["freq"] != inv_value[0]:
                if self.write_register(4, inv_value[0], SLAVE_ID_2):
                    prev_inv_values["freq"] = inv_value[0]
                    value_changed = True
                    #print(f"Frequency value changed to {inv_value[0]}")

            if prev_inv_values["accel"] != inv_value[1]:
                if self.write_register(6, inv_value[1], SLAVE_ID_2):
                    prev_inv_values["accel"] = inv_value[1]
                    value_changed = True
                    #print(f"Accel value changed to {inv_value[1]}")

            if prev_inv_values["decel"] != inv_value[2]:
                if self.write_register(7, inv_value[2], SLAVE_ID_2):
                    prev_inv_values["decel"] = inv_value[2]
                    value_changed = True
                    #print(f"Decel value changed to {inv_value[2]}")

            #if value_changed:
                #print("Values updated:", prev_inv_values)  # 디버깅용        
        prev_popup_status = False

        # 3번 주소에 1을 쓰고 초기화
        if value_changed and not prev_popup_status:  # value_changed가 True일 때만 실행
            success = self.write_register(3, 1, SLAVE_ID_1)
            if success:
                time.sleep(0.5)
                self.write_register(3, 0, SLAVE_ID_1)
                prev_popup_status = True
                #print(f"prev_popup_status set to True: {prev_popup_status}")  # 디버깅용
            #else:
            #    print("Failed to write to address 3")

        # 3번 주소에 값이 쓰였으면, prev_popup_status를 False로 리셋
        if prev_popup_status:
            prev_popup_status = False  # 한 번 쓰고 나면 다시 False로 리셋
            #print(f"prev_popup_status reset to False: {prev_popup_status}")  # 디버깅용

        # PLC에서 인버터 모니터링 값 읽기 (6번 ~ 9번 주소)
        inv_monitor = self.read_register(5, 5, SLAVE_ID_2, False) or [0, 0, 0, 0, 0]
        # print(f"inv_monitor 값: {inv_monitor}")

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
                success = self.write_register(addr, new_value, SLAVE_ID_1)
                if success:
                    #print(f"주소 {addr}에 {new_value} 쓰기 성공!")
                    prev_values[addr] = new_value  # 이전 값 업데이트
                #else:
                    #print(f"주소 {addr}에 {new_value} 쓰기 실패!")
                        
    def destroy_node(self):
        self.thread_running = False
        if self.thread.is_alive():
            self.thread.join()
            self.write_register(5,5057,SLAVE_ID_2)  #Stop
        super().destroy_node()

# PLC 연결 함수
def connect_to_plc():
    global PORT, BAUDRATE, PARITY, STOPBITS, BYTESIZE, TIMEOUT
    client = ModbusClient(
        framer='rtu',
        port=PORT,
        baudrate=BAUDRATE,
        parity=PARITY,
        stopbits=STOPBITS,
        bytesize=BYTESIZE,
        timeout=TIMEOUT
    )
    if client.connect():
        print("PLC와 연결 성공!")
        return client
    else:
        print("PLC와 연결 실패!")
        return None 

def main(args=None):
    rclpy.init(args=args)

    plc_client = connect_to_plc()
    if plc_client is None:
        print("PLC 연결 실패!")
        return
    
    plc_node = PlcNode(plc_client)

    try:
        rclpy.spin(plc_node)
    except KeyboardInterrupt:
        plc_node.get_logger().info("KeyboardInterrupt Stop.")
    finally:
        plc_node.destroy_node()

if __name__ == "__main__":
    main()