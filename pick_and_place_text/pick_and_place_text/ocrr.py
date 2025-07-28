import cv2
import pytesseract

# 이미지 불러오기
img = cv2.imread("/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_text/pick_and_place_text/abab.png")

# 텍스트 추출
text = pytesseract.image_to_string(img)
print(text)