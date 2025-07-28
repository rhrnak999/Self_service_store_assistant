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
from ultralytics import YOLO

BEEP_PATH = "/home/rokey/ros2_ws/barcode/qrbarcode_beep.mp3"
XLS_PATH = "/home/rokey/ros2_ws/barcode/cigarette_barcode.xls"
SALES_CSV_PATH = "/home/rokey/ros2_ws/barcode/sales_history.csv"
SCAN_COOLDOWN = 2
CAM_NUM = 0

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
        
        self.checkout_button = tk.Button(root, text="결제", font=("Arial", 14), command=self.checkout)
        self.checkout_button.pack(pady=10)
        
        self.root.bind('q', lambda event: self.back_to_main())
        self.root.bind('e', lambda event: self.confirm_order())

        # 수정된 부분: ROS 노드 전달
        self.scanner = BarcodeScanner(self, XLS_PATH, self.ros_node)
        self.scan_thread = Thread(target=self.scanner.scan)
        self.scan_thread.start()
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    # 나머지 메서드들은 기존과 동일
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
        if not self.sales:
            messagebox.showwarning("경고", "결제할 품목이 없습니다!")
            return
        
        total_price = sum(info['count'] * info['price'] for info in self.sales.values())
        confirm = messagebox.askyesno("결제 확인", f"총액: {total_price}원\n결제를 완료하시겠습니까?")
        if confirm:
            save_sales_history(self.sales)
            self.sales = {}
            for item in self.tree.get_children():
                self.tree.delete(item)
            self.total_label.config(text="총액: 0원")
            self.label.config(text="결제 완료! 바코드 스캔 대기 중...")
            messagebox.showinfo("완료", "결제가 완료되었습니다.")
    
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
# ROS2 바코드 서비스 노드 (개선된 버전)
class BarcodeServiceNode(Node):
    def __init__(self):
        super().__init__('barcode_service_node')
        self.service = self.create_service(Trigger, 'barcode_trigger', self.barcode_trigger_callback)
        self.publisher = self.create_publisher(String, 'barcode_topic', 10)
        
        # 바코드 스캐너와 공유할 상태
        self.barcode_detected = False
        self.latest_barcode = ""
        self.scanner_instance = None
        
        self.get_logger().info('바코드 서비스 노드가 시작되었습니다.')
    
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
        self.model = YOLO('/home/rokey/Downloads/bar.pt')

        
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
            
            results = self.model.predict(frame, conf=0.8, verbose=False)[0]
            boxes = results.boxes

            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf = float(box.conf[0])  # 신뢰도
                    cls_id = int(box.cls[0])   # 클래스 번호
                    label = self.model.names[cls_id] if hasattr(self.model, "names") else "object"

                    # 박스 크기 기우기
                    expand_pixels = 10
                    x1 = max(0, x1 - expand_pixels)
                    y1 = max(0, y1 - expand_pixels)
                    x2 = min(frame.shape[1], x2 + expand_pixels)
                    y2 = min(frame.shape[0], y2 + expand_pixels)

                    # 박스 그리기
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    barcode_roi = frame[y1:y2, x1:x2]

                    # yolo 이미지 전처리
                    gray = cv2.cvtColor(barcode_roi, cv2.COLOR_BGR2GRAY)
                    #blurred = cv2.GaussianBlur(gray, (3, 3), 0)
                    # thresh = cv2.adaptiveThreshold(
                    #     blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    #     cv2.THRESH_BINARY, 11, 2
                    # )

                    #decoded_objs = pyzbar.decode(thresh)
                    # decoded_objs = pyzbar.decode(blurred)
                    decoded_objs = pyzbar.decode(gray)

                    for code in decoded_objs:

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
########################
class Pay_ServiceNode(Node):
    def __init__(self):
        super().__init__('pay_service_node')
        self.service = self.create_service(Trigger, 'pay_service_trigger', self.pay_trigger_callback)

        # robot과 공유할 후불 버튼 클릭 상태 
        self.pay_condition = False
        self.latest_pay = ""
        self.pay_instance = None

        self.get_logger().info('후불 서비스 노드가 시작되었습니다.')
        #############
    def pay_trigger_callback(self, request, response):
        """PAY 트리거 서비스 콜백"""

################################################################
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