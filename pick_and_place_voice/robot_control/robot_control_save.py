import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init
from copy import deepcopy           #추가
import re                           #추가

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG

package_path = get_package_share_directory("pick_and_place_voice")
##########################################################################
from pick_and_place_interfaces.srv import PaymentCommand

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
# BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0
ox, oy  = 0, 0     #result 받아올 글로벌 상수 선언
quadrant = 0       #추가
tool_coord = [178.38, -90.11, -89.87]   #추가
ready_for_coor_list = [-20, 0, 20, 0, 0, 0]     #객체 탐지 후 근처거리로 이동하는 상대 좌표
grab_pos = [30, 0, 25, 0, 0, 0]                 #잡는 위치로 상대 이동 거리
JReady = [-3.57, -50.24, 107.06, -4.6, 52.94, 92.02]            #홈좌표(카메라 탐지 좌표)
LReady = [328.15, -41.05, 476.22, 172.35, -109.66, -91.64]
go_up_coord = [0, 0, 60, 0, 0, 0]           #단순 z 좌표 상대 이동
go_up_coord2 = [0, 0, -100, 0, 0, 0]        #툴좌표 상대 이동 좌표


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans, DR_MV_MOD_REL, DR_TOOL, set_singularity_handling, ikin, DR_AVOID, DR_BASE       #이모저모 추가
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


########### Robot Controller ############


class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.init_robot()

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

        # 바코드 서비스 클라이언트 추가
        self.barcode_client = self.create_client(Trigger, 'barcode_trigger')
        while not self.barcode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for barcode service...")

        ## call 실행
        self.robot_control()
    #####################
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
    ########################        
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

        msg = get_keyword_result.message.strip()                #앞뒤 공백 제거
        if "/" in msg:
            object_part, count_part = msg.split("/", 1)         #만약 / 존재시 한번만 분리
        else:
            object_part, count_part = msg, ""                   #없다면 분리없이 msg, 공백
        object_list = object_part.strip().split()               #앞뒤 공백제거, 공백별로 분리후 리스트 생성
        # count_part 없으면 각 1개씩
        import re
        cnt = re.sub(r"[\[\],'\"]", "", count_part).strip()     #[ ] , ' " 문자들 제거
        count_list = list(map(int, cnt.split())) if cnt else [1]*len(object_list)       #int로 전환, 빈값들 1로 지정



            #------------------------------------------
            #수정된 동작 반복문 방식--count만큼 우선 반복, 카운트 동안 target_pos 수행
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

            #------------------------------------------
            # for target in target_list:
            #     target_pos = self.get_target_pos(target)
            #     if target_pos is None:
            #         self.get_logger().warn("No target position")
            #     else:
            #         self.get_logger().info(f"target position: {target_pos}")
            #         self.pick_and_place_target(target_pos)
            #         self.init_robot()

        else:
            self.get_logger().warn(f"{get_keyword_result.message}")
            return

    def get_target_pos(self, target):
        import ast
        if isinstance(target, list):        #타겟이 list타입이면 첫번째 인자 수행
            target = target[0]
        elif isinstance(target, str):       #str 타입이라면
            try:
                parsed = ast.literal_eval(target)       #리터럴하게 재평가 혹은 재 확인
                if isinstance(parsed, list):            #평가 결과 리스트라면
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

            global ox, oy, quadrant                         #사분면 결정 저장
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

            target_pos = list(td_coord[:3]) + tool_coord        #툴 정렬 방향 결정
            return target_pos

        return None


    def init_robot(self):
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_target(self, target_pos):
        # target_pos[2] += 25
        # target_pos[0] += 30
        target_pos = [x+y for x,y in zip(target_pos, grab_pos)]
        set_singularity_handling(mode=DR_AVOID)

        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        # time.sleep(5)
        gripper.close_gripper(force_val=80)         #잡기

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


        movel(go_up_coord2,                 # 툴좌표 z 이동
            vel=VELOCITY,
            acc=ACC,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
        )
        mwait()

        #target_pos_up = trans(target_pos, [0, 0, 20, 0, 0, 0]).tolist()

        #movel(target_pos_up, vel=VELOCITY, acc=ACC)
        #mwait()

    def ready_for_grab(self, target_pos):
        sol = 0
        above_target = deepcopy(target_pos)
        above_target = [x+y for x, y in zip(above_target, ready_for_coor_list)]
        # above_target[0]-=20
        # above_target[2]+=20
        
        if quadrant == 1:
            sol = 2 
            self.get_logger().info("sol == 2")
        else:
            sol = 3
            self.get_logger().info("sol == 3")
        if sol == 0:
            self.get_logger().info("죽어")
            return
        above_target_j = ikin(above_target, sol).tolist()       #좌측 3 우측 2
        #bove_target_j = ikin(above_target, sol_space=3).tolist()       #좌측 3 우측 2
        # self.mj.move_j(above_target_j, vel=30, acc=30)
        cp1 = get_current_posx()[0]
        movej(above_target_j, vel=VELOCITY, acc=ACC)
        cp2 = get_current_posx()[0]
    
        if cp1 == cp2:
            self.get_logger().info("j이동 실패")
        else:
            self.get_logger().info("j이동 완료")

        # time.sleep(5)
###################################추가함##################################
    def go_barcode(self):
        #경유점
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
                #return  # 바코드 인식 시 함수 탈출
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
###################################추가함##################################

def main(args=None):
    node = RobotController()
    try:
        while rclpy.ok():
            node.robot_control()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()