import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init
from copy import deepcopy

from od_msg.srv import SrvDepthPosition
from ament_index_python.packages import get_package_share_directory
from pick_and_place_text.onrobot import RG

from std_srvs.srv import Trigger

package_path = get_package_share_directory("pick_and_place_text")

tool_dict = {0: "the one",
             1: "parliament", 
             2: "africa", 
             3: "dunhill",
             4: "bohem",
             5: "halla",
             6: "esse",
             7: "raison",
             8: "mevius",
             9: "this plus"}

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 20, 20
FORCE_THRESHOLD = 20
ox, oy  = 0, 0     #result 받아올 글로벌 상수 선언
quadrant = 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, get_tool_force, check_motion, task_compliance_ctrl, release_compliance_ctrl, trans, amovej,ikin, set_singularity_handling, posx, DR_MV_MOD_REL, DR_TOOL, DR_AVOID
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############

GRIPPER_NAME = "rg2"
TOOLCHANGER_IP = "192.168.1.1"
TOOLCHANGER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT)


########### Robot Controller ############
class Move_Js:
    def __init__(self):
        pass

    def move_j(self,pos,vel=30,acc=30):                    ##move로 바꿀때 각속도 설정하기
        flag=False
        while True:
            print("go_amove")
            amovej(pos, vel=vel, acc=acc)
            time.sleep(1.0)
            if check_motion()==0:   # 모션이 완료 된 경우
                print("최종 목표 위치 도달")
                break
            while True:
                fx, fy, fz, tx, ty, tz = get_tool_force()
                # fx, fy, fz, tx, ty, tz = fake_get_tool_force()
                total_force = (fx**2 + fy**2 + fz**2)**0.5

                if total_force > FORCE_THRESHOLD:
                    print(f"⚠ 외력 감지됨: {total_force:.2f} N → 모션 정지")
                    #MoveStop
                    # 순응 제어 활성화 (XYZ 방향만 허용)
                    task_compliance_ctrl([1, 1, 1, 0, 0, 0])
                    time.sleep(0.5)
                    release_compliance_ctrl()
                    time.sleep(0.5)
                    break
                if check_motion()==0:   # 모션이 완료 된 경우
                    flag=True
                    break
            if flag:
                print("최종 목표 위치 도달")
                break

class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.mj = Move_Js()
        self.init_robot()
        self.depth_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.depth_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for depth position service...")
        self.depth_request = SrvDepthPosition.Request()

         # 바코드 서비스 클라이언트 추가
        self.barcode_client = self.create_client(Trigger, 'barcode_trigger')
        while not self.barcode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for barcode service...")
        
        self.robot_control()

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T
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
        print("====================================")
        print("Available tools: ")
        print(
            "  0: the one\n 1: parliament\n 2: africa\n 3: dunhill\n 4: bohem\n 5: halla\n 6: esse\n 7: raison\n 8: mevius\n 9: this plus"
        )
        user_input = input("What do you want to bring?: ")
        if user_input.lower() == "q":
            self.get_logger().info("Quit the program...")
            sys.exit()

        if user_input:
            try:
                user_input_int = int(user_input)
                user_input = tool_dict.get(user_input_int, user_input)
            except ValueError:
                pass  # 변환 불가능하면 원래 문자열 유지
            self.depth_request.target = user_input
            self.get_logger().info("call depth position service with yolo")
            depth_future = self.depth_client.call_async(self.depth_request)
            rclpy.spin_until_future_complete(self, depth_future)

            if depth_future.result():
                result = depth_future.result().depth_position.tolist()
                self.get_logger().info(f" Received depth position: {result}")
                # self.get_logger().info(f"{type(result)}")
                
                if sum(result) == 0:
                    print("No target position")
                    return
                
                global ox, oy, quadrant                 # 글로벌 변수 불러오기
                ox = result[0]
                oy = result[1]         # ox, oy 에 좌표값 넣기 
                if ox>=0 and oy < 0:
                    quadrant = 1
                elif ox < 0 and oy <0:
                    quadrant = 2
                elif ox<0 and oy>=0:
                    quadrant = 3
                elif ox >= 0 and oy >= 0:
                    quadrant = 4

                gripper2cam_path = os.path.join(
                    package_path, "resource", "T_gripper2camera.npy"
                )
                robot_posx = get_current_posx()[0]
                td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

                if td_coord[2] and sum(td_coord) != 0:
                    td_coord[2] += -5  # DEPTH_OFFSET
                    td_coord[2] = max(td_coord[2], 2)  # MIN_DEPTH: float = 2.0

                target_pos = list(td_coord[:3]) + [178.38, -90.11, -89.87]

                self.get_logger().info(f"target position: {target_pos}")
                self.init_robot()
                self.ready_for_grab(target_pos)
                self.pick_and_place_target(target_pos)
                self.go_barcode()
                self.check_barcode()
                
        self.init_robot()

    def init_robot(self):
        JReady = [-3.57, -50.24, 107.06, -4.6, 52.94, 92.02]
        LReady = [328.15, -41.05, 476.22, 172.35, -109.66, -91.64]
        self.mj.move_j(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_target(self, target_pos):
        JReady = [-3.57, -50.24, 107.06, -4.6, 52.94, 92.02]
        LReady = [328.15, -41.05, 476.22, 172.35, -109.66, -91.64]
        target_pos[2] += 25
        target_pos[0] += 30
        set_singularity_handling(mode=DR_AVOID)

        movel(target_pos, vel=VELOCITY, acc=ACC) #잡으러 감
        mwait()
        gripper.close_gripper(force_val=80)         #잡기

        while gripper.get_status()[0]:
            time.sleep(0.1)
        while not gripper.get_status()[1]:
            self.get_logger().info("not grabbed")

        # grabbed_pos = get_current_posx()[0]
        # grabbed_pos[2] += 10

        movel([0, 0, 30, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL) # 위로 올림

        
        movel([0, 30, 0, 0, 0, 0],                 # 툴좌표 y 이동 # 위로 또 올림
            vel=VELOCITY,
            acc=ACC,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
        )
        mwait()

        movel([0, 0, -100, 0, 0, 0],                 # 툴좌표 z 이동 뒤로 뺌
            vel=VELOCITY,
            acc=ACC,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
        )
        mwait()

        # target_pos_up = trans(target_pos, [0, 0, 20, 0, 0, 0]).tolist()

        # movel(target_pos_up, vel=VELOCITY, acc=ACC)
        # mwait()


    def ready_for_grab(self, target_pos):
        sol = 0
        above_target = deepcopy(target_pos)
        above_target[0]-=20
        above_target[2]+=20
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
        self.mj.move_j(above_target_j, vel=30, acc=30)

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
                bar_pos = get_current_posx()[0]
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