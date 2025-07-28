import cv2
import pyzbar.pyzbar as pyzbar
from playsound import playsound
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QRBarcodePublisher(Node):
    def __init__(self):
        super().__init__('qrbarcode_publisher')
        self.publisher_ = self.create_publisher(String, 'qrbarcode_topic', 10)

        self.used_codes = []
        self.load_previous_data()

        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

    def load_previous_data(self):
        try:
            with open("qrbarcode_data.txt", "r", encoding="utf8") as f:
                self.used_codes = [line.strip() for line in f.readlines()]
        except FileNotFoundError:
            pass

    def run(self):
        while rclpy.ok():
            success, frame = self.cap.read()
            if not success:
                continue

            for code in pyzbar.decode(frame):
                my_code = code.data.decode('utf-8')
                if my_code not in self.used_codes:
                    print("인식 성공:", my_code)
                    playsound("qrbarcode_beep.mp3")
                    self.used_codes.append(my_code)

                    with open("qrbarcode_data.txt", "a", encoding="utf8") as f:
                        f.write(my_code + '\n')

                    msg = String()
                    msg.data = my_code
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"발행된 QR/barcode: {my_code}")
                else:
                    print("이미 인식된 코드입니다!")
                    playsound("qrbarcode_beep.mp3")

            cv2.imshow('QRcode Barcode Scan', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = QRBarcodePublisher()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
