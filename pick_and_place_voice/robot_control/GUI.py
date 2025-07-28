import cv2
import pyzbar.pyzbar as pyzbar
from playsound import playsound
import pandas as pd
import time
import tkinter as tk
from tkinter import ttk, messagebox
from threading import Thread
import os
import csv
from datetime import datetime
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
from pick_and_place_interfaces.srv import PaymentCommand


BEEP_PATH = "/home/rokey/ros2_ws/barcode/qrbarcode_beep.mp3"
XLS_PATH = "/home/rokey/ros2_ws/barcode/cigarette_barcode.xls"
SALES_CSV_PATH = "/home/rokey/ros2_ws/barcode/sales_history.csv"
SCAN_COOLDOWN = 2
CAM_NUM = 6

# 엑셀 파일 파싱
def load_cigarette_data(excel_file):
    try:
        df = pd.read_excel(excel_file, dtype={'barcode': str})
        required_columns = ['barcode', 'descr', 'money1']
        missing_columns = [col for col in required_columns if col not in df.columns]
        if missing_columns:
            raise KeyError(f"엑셀 파일에 다음 열이 누락되었습니다: {missing_columns}. 실제 열: {df.columns.tolist()}")

        products = {}
        for _, row in df.iterrows():
            barcode = str(row['barcode'])
            products[barcode] = {
                'name': row['descr'],
                'price': int(row['money1'])
            }
        return products
    except FileNotFoundError:
        raise FileNotFoundError(f"엑셀 파일을 찾을 수 없습니다: {excel_file}")
    except Exception as e:
        raise Exception(f"엑셀 파일 처리 중 오류: {str(e)}")

# 구매 내역 CSV 저장
def save_sales_history(sales, filename="sales_history.csv"):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    grand_total = sum(info['count'] * info['price'] for info in sales.values())
    
    with open(filename, 'a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        if os.path.getsize(filename) == 0:
            writer.writerow(['Date', 'Barcode', 'ProductName', 'Count', 'Price', 'Total', 'GrandTotal'])
        for barcode, info in sales.items():
            total = info['count'] * info['price']
            writer.writerow([timestamp, barcode, info['name'], info['count'], info['price'], total, grand_total])

# 메인 GUI 클래스 수정
class MainGUI:
    def __init__(self, root, ros_node=None):
        self.root = root
        self.ros_node = ros_node
        self.root.title("담배 판매 시스템 - 메인")
        self.root.geometry("800x600")
        
        self.label = tk.Label(root, text="관리자 메뉴", font=("Arial", 20, "bold"))
        self.label.pack(pady=20)
        
        self.scan_button = tk.Button(root, text="바코드 스캔", font=("Arial", 14), command=self.open_scan)
        self.scan_button.pack(pady=10)
        
        self.inventory_button = tk.Button(root, text="재고 관리", font=("Arial", 14), command=self.open_inventory)
        self.inventory_button.pack(pady=10)
        
        self.sales_button = tk.Button(root, text="금일 판매량", font=("Arial", 14), command=self.open_sales)
        self.sales_button.pack(pady=10)
        
        self.root.bind('<Control-Shift-A>', lambda event: self.open_scan())
        
    def open_scan(self):
        self.root.withdraw()
        scan_window = tk.Toplevel(self.root)
        SalesGUI(scan_window, self.root, self.ros_node)  # ROS 노드 전달
        
    def open_inventory(self):
        messagebox.showinfo("재고 관리", "재고 관리 기능은 아직 구현되지 않았습니다.")
        
    def open_sales(self):
        self.root.withdraw()
        sales_window = tk.Toplevel(self.root)
        SalesReportGUI(sales_window, self.root)

# 판매 내역 GUI 클래스
class SalesReportGUI:
    def __init__(self, root, main_root):
        self.root = root
        self.main_root = main_root
        self.root.title("금일 판매량")
        self.root.geometry("800x600")
        
        self.label = tk.Label(root, text="금일 판매량", font=("Arial", 16, "bold"))
        self.label.pack(pady=10)
        
        self.tree = ttk.Treeview(root, columns=("Date", "GrandTotal"), show="headings")
        self.tree.heading("Date", text="구매 시간")
        self.tree.heading("GrandTotal", text="총액")
        self.tree.column("Date", width=300)
        self.tree.column("GrandTotal", width=200)
        self.tree.pack(pady=10, padx=10, fill="both", expand=True)
        self.tree.bind('<Double-1>', self.show_details)
        
        self.back_button = tk.Button(root, text="메인으로 돌아가기", font=("Arial", 14), command=self.back_to_main)
        self.back_button.pack(pady=10)
        
        self.load_sales()
        
    def load_sales(self):
        try:
            df = pd.read_csv("sales_history.csv")
            df['Date'] = pd.to_datetime(df['Date'])
            today = datetime.now().date()
            df_today = df[df['Date'].dt.date == today]
            
            grouped = df_today.groupby('Date')['GrandTotal'].first().reset_index()
            for _, row in grouped.iterrows():
                self.tree.insert("", "end", values=(row['Date'].strftime("%Y-%m-%d %H:%M:%S"), f"{row['GrandTotal']}원"))
        except FileNotFoundError:
            self.label.config(text="판매 내역이 없습니다.")
        
    def show_details(self, event):
        selected_item = self.tree.selection()
        if not selected_item:
            return
        
        date = self.tree.item(selected_item)['values'][0]
        details_window = tk.Toplevel(self.root)
        details_window.title(f"구매 내역 상세 - {date}")
        details_window.geometry("800x600")
        
        tree = ttk.Treeview(details_window, columns=("Barcode", "ProductName", "Count", "Price", "Total"), show="headings")
        tree.heading("Barcode", text="바코드")
        tree.heading("ProductName", text="상품명")
        tree.heading("Count", text="수량")
        tree.heading("Price", text="단가")
        tree.heading("Total", text="합계")
        tree.column("Barcode", width=150)
        tree.column("ProductName", width=200)
        tree.column("Count", width=100)
        tree.column("Price", width=100)
        tree.column("Total", width=100)
        tree.pack(pady=10, padx=10, fill="both", expand=True)
        
        df = pd.read_csv("sales_history.csv")
        df['Date'] = pd.to_datetime(df['Date'])
        details = df[df['Date'].dt.strftime("%Y-%m-%d %H:%M:%S") == date]
        for _, row in details.iterrows():
            tree.insert("", "end", values=(row['Barcode'], row['ProductName'], row['Count'], f"{row['Price']}원", f"{row['Total']}원"))
        
    def back_to_main(self):
        self.root.destroy()
        self.main_root.deiconify()

# 기존 GUI 클래스들은 그대로 유지하되, SalesGUI만 수정
class SalesGUI:
    def __init__(self, root, main_root, ros_node=None):
        self.root = root
        self.main_root = main_root
        self.ros_node = ros_node
        self.root.title("담배 판매 시스템 - 스캔")
        self.root.geometry("800x600")
        
        self.sales = {}
        
        self.label = tk.Label(root, text="바코드 스캔 대기 중...", font=("Arial", 14))
        self.label.pack(pady=10)
        
        self.tree = ttk.Treeview(root, columns=("Name", "Count", "Price", "Total"), show="headings")
        self.tree.heading("Name", text="상품명")
        self.tree.heading("Count", text="수량")
        self.tree.heading("Price", text="단가")
        self.tree.heading("Total", text="합계")
        self.tree.column("Name", width=300)
        self.tree.column("Count", width=100)
        self.tree.column("Price", width=100)
        self.tree.column("Total", width=100)
        self.tree.pack(pady=10, padx=10, fill="both", expand=True)
        
        self.total_label = tk.Label(root, text="총액: 0원", font=("Arial", 16, "bold"))
        self.total_label.pack(pady=10)
        
        # 버튼 프레임 생성 (결제와 환불 버튼을 나란히 배치)
        button_frame = tk.Frame(root)
        button_frame.pack(pady=10)
        
        self.checkout_button = tk.Button(button_frame, text="결제", font=("Arial", 14), 
                                       command=self.checkout, bg="lightgreen")
        self.checkout_button.pack(side=tk.LEFT, padx=10)
        
        # 환불 버튼 추가
        self.refund_button = tk.Button(button_frame, text="환불", font=("Arial", 14), 
                                     command=self.refund, bg="lightcoral")
        self.refund_button.pack(side=tk.LEFT, padx=10)
        
        # 메인으로 돌아가기 버튼
        self.back_button = tk.Button(root, text="메인으로 돌아가기", font=("Arial", 14), 
                                   command=self.back_to_main)
        self.back_button.pack(pady=10)
        
        self.root.bind('q', lambda event: self.back_to_main())
        self.root.bind('e', lambda event: self.confirm_order())

        # 수정된 부분: ROS 노드 전달
        self.scanner = BarcodeScanner(self, XLS_PATH, self.ros_node)
        self.scan_thread = Thread(target=self.scanner.scan)
        self.scan_thread.start()
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def update_sales(self, barcode, product_info):
        if barcode in self.sales:
            self.sales[barcode]['count'] += 1
        else:
            self.sales[barcode] = {
                'name': product_info['name'],
                'count': 1,
                'price': product_info['price']
            }
        
        for item in self.tree.get_children():
            self.tree.delete(item)
        
        total_price = 0
        for barcode, info in self.sales.items():
            total = info['count'] * info['price']
            total_price += total
            self.tree.insert("", "end", values=(info['name'], info['count'], f"{info['price']}원", f"{total}원"))
        
        self.total_label.config(text=f"총액: {total_price}원")
        self.label.config(text=f"인식 성공: {product_info['name']}")
    
    def checkout(self):
        """결제 처리 - 로봇에 결제 명령 전송 포함"""
        if not self.sales:
            messagebox.showwarning("경고", "결제할 품목이 없습니다!")
            return
        
        total_price = sum(info['count'] * info['price'] for info in self.sales.values())
        confirm = messagebox.askyesno("결제 확인", f"총액: {total_price}원\n결제를 완료하시겠습니까?")
        
        if confirm:
            # 로컬 결제 처리
            save_sales_history(self.sales)
            self.sales = {}
            for item in self.tree.get_children():
                self.tree.delete(item)
            self.total_label.config(text="총액: 0원")
            
            # 로봇에 결제 명령 전송
            if self.ros_node:
                self.label.config(text="로봇 결제 처리 중...")
                self.checkout_button.config(state=tk.DISABLED)
                self.refund_button.config(state=tk.DISABLED)
                
                # 별도 스레드에서 로봇 결제 처리
                payment_thread = Thread(target=self._process_robot_payment)
                payment_thread.start()
            else:
                self.label.config(text="결제 완료! 바코드 스캔 대기 중...")
                messagebox.showinfo("완료", "결제가 완료되었습니다.")
    
    def _process_robot_payment(self):
        """로봇 결제 처리 (별도 스레드)"""
        try:
            success = self.ros_node.call_payment_service("pay")
            
            # GUI 업데이트는 메인 스레드에서
            self.root.after(0, self._update_payment_result, success, "결제")
            
        except Exception as e:
            self.root.after(0, self._update_payment_result, False, "결제", str(e))
    
    def refund(self):
        """환불 처리 - 로봇에 환불 명령 전송"""
        confirm = messagebox.askyesno("환불 확인", "환불 처리를 진행하시겠습니까?")
        
        if confirm:
            if self.ros_node:
                self.label.config(text="로봇 환불 처리 중...")
                self.checkout_button.config(state=tk.DISABLED)
                self.refund_button.config(state=tk.DISABLED)
                
                # 별도 스레드에서 로봇 환불 처리
                refund_thread = Thread(target=self._process_robot_refund)
                refund_thread.start()
            else:
                messagebox.showinfo("완료", "환불이 완료되었습니다.")
    
    def _process_robot_refund(self):
        """로봇 환불 처리 (별도 스레드)"""
        try:
            success = self.ros_node.call_payment_service("refund")
            
            # GUI 업데이트는 메인 스레드에서
            self.root.after(0, self._update_payment_result, success, "환불")
            
        except Exception as e:
            self.root.after(0, self._update_payment_result, False, "환불", str(e))
    
    def _update_payment_result(self, success, operation, error_msg=None):
        """결제/환불 결과 GUI 업데이트"""
        self.checkout_button.config(state=tk.NORMAL)
        self.refund_button.config(state=tk.NORMAL)
        
        if success:
            self.label.config(text=f"로봇 {operation} 완료! 바코드 스캔 대기 중...")
            messagebox.showinfo("완료", f"로봇 {operation}이 완료되었습니다.")
        else:
            error_text = f"로봇 {operation} 실패"
            if error_msg:
                error_text += f": {error_msg}"
            self.label.config(text=f"{error_text}. 바코드 스캔 대기 중...")
            messagebox.showerror("오류", error_text)
    
    def confirm_order(self):
        if not self.sales:
            messagebox.showwarning("경고", "주문이 없습니다!")
            return
        
        order_text = ", ".join(f"{info['name']} {info['count']}개" for barcode, info in self.sales.items())
        confirm = messagebox.askyesno("주문 확인", f"주문하신 상품: {order_text}\n맞으신가요?")
        print(f"주문 확인 결과: {confirm}")
        self.label.config(text=f"주문 확인: {order_text}")
        
    def back_to_main(self):
        self.scanner.stop()
        self.scan_thread.join()
        self.root.destroy()
        self.main_root.deiconify()
        
    def on_closing(self):
        self.scanner.stop()
        self.scan_thread.join()
        self.root.destroy()
        self.main_root.destroy()

#########################################################333
# ROS2 바코드 서비스 노드 (PaymentCommand 클라이언트 추가)
class BarcodeServiceNode(Node):
    def __init__(self):
        super().__init__('barcode_service_node')
        
        # 기존 바코드 서비스
        self.service = self.create_service(Trigger, 'barcode_trigger', self.barcode_trigger_callback)
        self.publisher = self.create_publisher(String, 'barcode_topic', 10)
        
        # PaymentCommand 클라이언트 추가
        self.payment_client = self.create_client(PaymentCommand, 'payment_command')
        
        # 바코드 스캐너와 공유할 상태
        self.barcode_detected = False
        self.latest_barcode = ""
        self.scanner_instance = None
        
        self.get_logger().info('바코드 서비스 노드가 시작되었습니다.')
        
        # PaymentCommand 서비스 대기
        self._wait_for_payment_service()
    
    def _wait_for_payment_service(self):
        """PaymentCommand 서비스 대기 (비블로킹)"""
        def check_service():
            if self.payment_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('PaymentCommand 서비스에 연결되었습니다.')
                return
            else:
                self.get_logger().info('PaymentCommand 서비스를 기다리는 중...')
                self.create_timer(2.0, check_service)
        
        check_service()
    
    def set_scanner(self, scanner):
        """스캐너 인스턴스 연결"""
        self.scanner_instance = scanner
    
    def barcode_trigger_callback(self, request, response):
        """바코드 트리거 서비스 콜백"""
        try:
            if self.scanner_instance and self.scanner_instance.scan_value:
                response.success = True
                response.message = f"바코드 인식됨: {self.scanner_instance.latest_barcode}"
                
                # 토픽으로도 발행
                msg = String()
                msg.data = self.scanner_instance.latest_barcode
                self.publisher.publish(msg)
                
                # 서비스 호출 후 상태 리셋 (중요!)
                self.scanner_instance.scan_value = False

                self.get_logger().info(f'바코드 서비스 응답: {response.message}')
            else:
                response.success = False
                response.message = "바코드가 인식되지 않음"
                
        except Exception as e:
            response.success = False
            response.message = f"서비스 오류: {str(e)}"
            self.get_logger().error(f'서비스 오류: {str(e)}')
            
        return response
    
    def call_payment_service(self, command):
        """PaymentCommand 서비스 호출"""
        if not self.payment_client.service_is_ready():
            self.get_logger().error('PaymentCommand 서비스가 준비되지 않았습니다.')
            return False
        
        try:
            request = PaymentCommand.Request()
            request.command = command
            
            self.get_logger().info(f'로봇에 {command} 명령 전송 중...')
            
            # 동기적 호출 (타임아웃 30초)
            future = self.payment_client.call_async(request)
            
            # 결과 대기
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 30.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f'로봇 {command} 성공: {response.message}')
                    return True
                else:
                    self.get_logger().error(f'로봇 {command} 실패: {response.message}')
                    return False
            else:
                self.get_logger().error(f'로봇 {command} 명령 타임아웃')
                return False
                
        except Exception as e:
            self.get_logger().error(f'PaymentCommand 서비스 호출 오류: {str(e)}')
            return False

################################################################
# 바코드 스캔 클래스 (개선된 버전)
class BarcodeScanner:
    def __init__(self, gui, excel_file, ros_node=None):
        self.gui = gui
        self.ros_node = ros_node
        try:
            self.products = load_cigarette_data(excel_file)
        except Exception as e:
            self.gui.label.config(text=f"오류: {str(e)}")
            raise
        
        self.last_scan_time = 0
        self.scan_cooldown = SCAN_COOLDOWN
        self.cap = cv2.VideoCapture(CAM_NUM, cv2.CAP_V4L2)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        self.running = True
        
        # ROS2 서비스용 상태
        self.scan_value = False
        self.latest_barcode = ""
        
        # ROS 노드에 스캐너 등록
        if self.ros_node:
            self.ros_node.set_scanner(self)
        
    def scan(self):#scannnnnnnn
        """바코드 스캔 메인 루프"""
        while self.running:
            success, frame = self.cap.read()
            if not success or frame is None:
                self.gui.label.config(text="카메라 연결 실패")
                continue
                
            for code in pyzbar.decode(frame):
                barcode = code.data.decode('utf-8')
                
                if barcode not in self.products:
                    self.gui.label.config(text=f"알 수 없는 바코드: {barcode}")
                    print(f"알 수 없는 바코드: {barcode}")
                    playsound(BEEP_PATH)
                    continue
                
                current_time = time.time()
                if current_time - self.last_scan_time >= self.scan_cooldown:
                    self.gui.update_sales(barcode, self.products[barcode])
                    print(f"인식 성공: {barcode} ({self.products[barcode]['name']})")
                    playsound(BEEP_PATH)
                    self.last_scan_time = current_time
                    
                    # ROS2 서비스용 상태 업데이트
                    self.scan_value = True
                    print(f"value값 : {self.scan_value}")
                    self.latest_barcode = barcode
                
            cv2.imshow('QRcode Barcode Scan', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
           
        self.cap.release()
        cv2.destroyAllWindows()
        
    def stop(self):
        self.running = False

# 메인 실행 (수정된 버전)
if __name__ == "__main__":
    rclpy.init()
    
    # ROS2 노드 생성
    barcode_service_node = BarcodeServiceNode()
    
    # 멀티스레드 실행자 생성
    executor = MultiThreadedExecutor()
    executor.add_node(barcode_service_node)
    
    # ROS2 스핀을 별도 스레드에서 실행
    ros_thread = Thread(target=executor.spin)
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        # GUI 실행
        root = tk.Tk()
        gui = MainGUI(root, barcode_service_node)
        root.bind('<Control-Shift-A>', lambda event: gui.open_scan())
        root.mainloop()
    finally:
        # 종료 처리
        executor.shutdown()
        barcode_service_node.destroy_node()
        rclpy.shutdown()