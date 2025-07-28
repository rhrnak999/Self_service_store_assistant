import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread, Event
import DR_init
from copy import deepcopy
import re

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG

package_path = get_package_share_directory("pick_and_place_voice")
#########################################################
from pick_and_place_interfaces.srv import PaymentCommand
#########################################################

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0
ox, oy = 0, 0
quadrant = 0
tool_coord = [178.38, -90.11, -89.87]
ready_for_coor_list = [-20, 0, 20, 0, 0, 0]
grab_pos = [30, 0, 25, 0, 0, 0]
JReady = [-3.57, -50.24, 107.06, -4.6, 52.94, 92.02]
LReady = [328.15, -41.05, 476.22, 172.35, -109.66, -91.64]
go_up_coord = [0, 0, 60, 0, 0, 0]
go_up_coord2 = [0, 0, -100, 0, 0, 0]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans, DR_MV_MOD_REL, DR_TOOL, set_singularity_handling, ikin, DR_AVOID, DR_BASE
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

########### 새로운 결제 서비스 처리 클래스 ############
class PaymentServiceHandler:
    def __init__(self, robot_controller):
        self.robot_controller = robot_controller
        self.payment_requested = False
        self.payment_command = ""
        self.payment_result = None
        self.payment_event = Event()
        self.running = True
        
    def request_payment(self, command):
        """결제/환불 요청 처리"""
        self.payment_command = command
        self.payment_requested = True
        self.payment_result = None
        self.payment_event.clear()
        
        # 결과 대기 (타임아웃 30초)
        if self.payment_event.wait(timeout=100.0):
            success, message = self.payment_result
            return success, message
        else:
            return False, "결제/환불 처리 타임아웃"
    
    def payment_service_thread(self):
        """결제 서비스 처리 스레드"""
        self.robot_controller.get_logger().info("🔄 결제 서비스 스레드 시작")
        
        while self.running:
            if self.payment_requested:
                try:
                    command = self.payment_command.lower()
                    self.robot_controller.get_logger().info(f'🤖 결제/환불 명령 처리 시작: {command}')
                    
                    if command == "pay":
                        success = self.robot_controller.perform_payment_action()
                        if success:
                            self.payment_result = (True, "결제 완료: 로봇이 결제 동작을 수행했습니다.")
                        else:
                            self.payment_result = (False, "결제 실패: 로봇 동작 중 오류가 발생했습니다.")
                            
                    elif command == "refund":
                        success = self.robot_controller.perform_refund_action()
                        if success:
                            self.payment_result = (True, "환불 완료: 로봇이 환불 동작을 수행했습니다.")
                        else:
                            self.payment_result = (False, "환불 실패: 로봇 동작 중 오류가 발생했습니다.")
                    else:
                        self.payment_result = (False, f"알 수 없는 명령: {command}")
                        
                    self.payment_requested = False
                    self.payment_event.set()
                    
                except Exception as e:
                    self.payment_result = (False, f"결제/환불 처리 오류: {str(e)}")
                    self.payment_requested = False
                    self.payment_event.set()
                    self.robot_controller.get_logger().error(f'결제/환불 처리 오류: {str(e)}')
            
            time.sleep(0.1)  # CPU 사용량 조절
        
        self.robot_controller.get_logger().info("🔄 결제 서비스 스레드 종료")
    
    def stop(self):
        """결제 서비스 종료"""
        self.running = False

########### Robot Controller ############
class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.init_robot()

        # 기존 서비스 클라이언트들
        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position"
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

        # 바코드 서비스 클라이언트
        self.barcode_client = self.create_client(Trigger, 'barcode_trigger')
        while not self.barcode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for barcode service...")

        # 결제 서비스 핸들러 생성 및 스레드 시작
        self.payment_handler = PaymentServiceHandler(self)
        self.payment_thread = Thread(target=self.payment_handler.payment_service_thread)
        self.payment_thread.daemon = True
        self.payment_thread.start()

        # 결제/환불 서비스 서버 생성 (스레드 기반으로 처리)
        self.payment_service = self.create_service(
            PaymentCommand, 
            'payment_service', 
            self.payment_callback
        )
        self.get_logger().info("✅ 결제/환불 서비스 서버가 시작되었습니다.")

    def payment_callback(self, request, response):
        """결제/환불 서비스 콜백 - 스레드로 처리 요청"""
        try:
            command = request.command.lower()
            self.get_logger().info(f'📞 결제/환불 서비스 요청 수신: {command}')
            
            # 스레드 기반 결제 처리 요청
            success, message = self.payment_handler.request_payment(command)
            
            response.success = success
            response.message = message
            
            if success:
                self.get_logger().info(f'✅ {command} 완료')
            else:
                self.get_logger().error(f'❌ {command} 실패: {message}')
                
        except Exception as e:
            response.success = False
            response.message = f"서비스 콜백 오류: {str(e)}"
            self.get_logger().error(f'서비스 콜백 오류: {str(e)}')
            
        return response

    def call_barcode_service(self):
        """바코드 서비스 호출"""
        request = Trigger.Request()
        future = self.barcode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            result = future.result()
            self.get_logger().info(f"바코드 서비스 응답: {result.message}")
            return result.success
        else:
            self.get_logger().error("바코드 서비스 호출 실패")
            return False

    def perform_payment_action(self):
        """실제 결제 로봇 동작"""
        try:
            self.get_logger().info('🤖 로봇 결제 동작 시작...')
            
            # 결제 동작 시퀀스
            # 1. 안전한 위치로 이동
            movej(JReady, vel=30, acc=30)
            mwait()
            
            # 2. 결제 포즈로 이동 (예: 카드 리더기 쪽으로 팔 뻗기)
            payment_pose = [-20.0, -30.0, 90.0, 0.0, 60.0, 0.0]
            movej(payment_pose, vel=20, acc=20)
            mwait()
            
            # 3. 결제 제스처 (예: 카드 터치 시뮬레이션)
            # 앞으로 조금 이동
            payment_touch = [10, 0, 0, 0, 0, 0]
            movel(payment_touch, vel=10, acc=10, mod=DR_MV_MOD_REL)
            mwait()
            time.sleep(1)  # 카드 터치 대기
            
            # 뒤로 조금 이동
            movel([-10, 0, 0, 0, 0, 0], vel=10, acc=10, mod=DR_MV_MOD_REL)
            mwait()
            
            # 4. 홈 위치로 복귀
            movej(JReady, vel=30, acc=30)
            mwait()
            
            self.get_logger().info('💳 결제 동작 완료')
            return True
            
        except Exception as e:
            self.get_logger().error(f'결제 동작 실패: {str(e)}')
            return False

    def perform_refund_action(self):
        """실제 환불 로봇 동작"""
        try:
            self.get_logger().info('🤖 로봇 환불 동작 시작...')
            
            # 환불 동작 시퀀스
            # 1. 안전한 위치로 이동
            movej(JReady, vel=30, acc=30)
            mwait()
            
            # 2. 환불 포즈로 이동 (예: 환불 버튼 쪽으로)
            refund_pose = [20.0, -40.0, 100.0, 0.0, 40.0, 180.0]
            movej(refund_pose, vel=20, acc=20)
            mwait()
            
            # 3. 환불 제스처 (예: 환불 버튼 누르기)
            # 아래로 이동 (버튼 누르기)
            button_press = [0, 0, -15, 0, 0, 0]
            movel(button_press, vel=5, acc=5, mod=DR_MV_MOD_REL)
            mwait()
            time.sleep(0.5)  # 버튼 누르기 대기
            
            # 위로 이동 (버튼 떼기)
            movel([0, 0, 15, 0, 0, 0], vel=5, acc=5, mod=DR_MV_MOD_REL)
            mwait()
            
            # 4. 확인 제스처 (손 흔들기)
            for _ in range(2):
                movel([0, 20, 0, 0, 0, 0], vel=15, acc=15, mod=DR_MV_MOD_REL)
                mwait()
                movel([0, -20, 0, 0, 0, 0], vel=15, acc=15, mod=DR_MV_MOD_REL)
                mwait()
            
            # 5. 홈 위치로 복귀
            movej(JReady, vel=30, acc=30)
            mwait()
            
            self.get_logger().info('💰 환불 동작 완료')
            return True
            
        except Exception as e:
            self.get_logger().error(f'환불 동작 실패: {str(e)}')
            return False

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        # 좌표 변환 (그리퍼 → 베이스)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def robot_control(self):
        target_list = []
        self.get_logger().info("call get_keyword service")
        self.get_logger().info("say 'Hello Rokey' and speak what you want to pick up")
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self, get_keyword_future)
        if get_keyword_future.result().success:
            get_keyword_result = get_keyword_future.result()

        msg = get_keyword_result.message.strip()
        if "/" in msg:
            object_part, count_part = msg.split("/", 1)
        else:
            object_part, count_part = msg, ""
        object_list = object_part.strip().split()
        
        import re
        cnt = re.sub(r"[\[\],'\"]", "", count_part).strip()
        count_list = list(map(int, cnt.split())) if cnt else [1]*len(object_list)

        # 수정된 동작 반복문 방식
        for target, count in zip(object_list, count_list):
            for _ in range(count):
                target_pos = self.get_target_pos(target)
                if target_pos is None:
                    self.get_logger().warn(f"Failed to find position for {target}")
                else:
                    self.ready_for_grab(target_pos)
                    self.pick_and_place_target(target_pos)
                    self.go_barcode()
                    self.check_barcode()
                self.init_robot()
        else:
            self.get_logger().warn(f"{get_keyword_result.message}")
            return

    def get_target_pos(self, target):
        import ast
        if isinstance(target, list):
            target = target[0]
        elif isinstance(target, str):
            try:
                parsed = ast.literal_eval(target)
                if isinstance(parsed, list):
                    target = parsed[0]
            except (ValueError, SyntaxError):
                pass

        self.get_position_request.target = target
        self.get_logger().info(f"call depth position service with object_detection node for target: {target}")

        get_position_future = self.get_position_client.call_async(self.get_position_request)
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

            global ox, oy, quadrant
            ox, oy = result[0], result[1]
            if ox >= 0 and oy < 0:
                quadrant = 1
            elif ox < 0 and oy < 0:
                quadrant = 2
            elif ox < 0 and oy >= 0:
                quadrant = 3
            elif ox >= 0 and oy >= 0:
                quadrant = 4

            gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)

            target_pos = list(td_coord[:3]) + tool_coord
            return target_pos

        return None

    def init_robot(self):
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_target(self, target_pos):
        target_pos = [x+y for x,y in zip(target_pos, grab_pos)]
        set_singularity_handling(mode=DR_AVOID)

        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper(force_val=80)

        while gripper.get_status()[0]:
            time.sleep(0.1)
        while not gripper.get_status()[1]:
            self.get_logger().info("not grabbed")

        movel(
            go_up_coord, 
              vel=VELOCITY, 
              acc=ACC, 
              mod=DR_MV_MOD_REL
              )
        mwait()

        movel(go_up_coord2,
            vel=VELOCITY,
            acc=ACC,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
        )
        mwait()

    def ready_for_grab(self, target_pos):
        sol = 0
        above_target = deepcopy(target_pos)
        above_target = [x+y for x, y in zip(above_target, ready_for_coor_list)]
        
        if quadrant == 1:
            sol = 2 
            self.get_logger().info("sol == 2")
        else:
            sol = 3
            self.get_logger().info("sol == 3")
        if sol == 0:
            self.get_logger().info("죽어")
            return
        above_target_j = ikin(above_target, sol).tolist()
        cp1 = get_current_posx()[0]
        movej(above_target_j, vel=VELOCITY, acc=ACC)
        cp2 = get_current_posx()[0]
    
        if cp1 == cp2:
            self.get_logger().info("j이동 실패")
        else:
            self.get_logger().info("j이동 완료")

    def go_barcode(self):
        # 경유점
        movej(pos=[-3.54, -21.07, 115.16, -4.6, -3.74, 92.02], 
              vel = 20,
              acc = 20)
        mwait()
        movej(pos = [-35.36, 47.53, 23.54, 0.28, 107.89, 60.16], vel =20, acc=20)
        mwait()

    def check_barcode(self):
        self.get_logger().info("바코드 인식을 시도합니다...")
        
        while True:
            # 바코드 서비스 호출하여 인식 여부 확인
            barcode_detected = self.call_barcode_service()
            
            if barcode_detected:
                self.get_logger().info("바코드가 인식되었습니다! 함수를 종료합니다.")
                barcode_detected = False
                break
            else:
                self.get_logger().info("바코드가 인식되지 않았습니다. 미세 조정을 수행합니다.")
                bar_pos = get_current_posx(ref=DR_BASE)[0]
                bar_pos[0] -= 20
                movel(bar_pos, vel = 5, acc = 5)
                mwait()
                bar_pos[0] += 20
                movel(bar_pos, vel = 5, acc = 5)
                mwait()
                
                # 잠시 대기 후 다시 시도 (너무 빠른 반복 방지)
                time.sleep(0.5)

        movej(pos = [-29.62, -3.3, 99.49, -0.33, 83.61, 5.74], vel = 10, acc = 10)
        movej(pos = [-84.21, 11.79, 103.71, 0.1, 64.51, 5.74], vel =10, acc=10)
        gripper.open_gripper()
        return

    def cleanup(self):
        """정리 작업"""
        self.payment_handler.stop()
        if self.payment_thread.is_alive():
            self.payment_thread.join(timeout=2.0)

def main(args=None):
    # 멀티스레드 실행자 생성
    executor = MultiThreadedExecutor(num_threads=4)
    
    node = RobotController()
    executor.add_node(node)
    
    try:
        # 픽앤플레이스 동작을 별도 스레드에서 실행
        control_thread = Thread(target=node.robot_control)
        control_thread.start()
        
        # ROS2 서비스들을 메인 스레드에서 처리
        executor.spin()
        
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트 감지, 종료 중...")
    finally:
        node.cleanup()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()