import cv2
import pyzbar.pyzbar as pyzbar
from playsound import playsound
#토픽 발행을 위한 임포트
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


used_codes = []
data_list = []

try:
    f = open("qrbarcode_data.txt", "r", encoding="utf8")
    data_list = f.readlines()
except FileNotFoundError:
    pass
else:
    f.close()

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

for i in data_list:
    used_codes.append(i.rstrip('\n'))

while True:
    success, frame = cap.read()

    for code in pyzbar.decode(frame):
        cv2.imwrite('qrbarcode_image.png', frame)
        my_code = code.data.decode('utf-8')
        if my_code not in used_codes:
            print("인식 성공 : ", my_code)
            playsound("qrbarcode_beep.mp3")
            used_codes.append(my_code)

            f2 = open("qrbarcode_data.txt", "a", encoding="utf8")
            f2.write(my_code+'\n')
            f2.close()
        elif my_code in used_codes:
            print("이미 인식된 코드 입니다.!!!")
            playsound("qrbarcode_beep.mp3")
        else:
            pass

    cv2.imshow('QRcode Barcode Scan', frame)

    cv2.waitKey(1)

