import cv2
import numpy as np
import robomaster
from robomaster import robot
from robomaster import vision

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="sta")

ep_camera = ep_robot.camera

ep_camera.start_video_stream(display=False)

while True:
    img = ep_camera.read_cv2_image()
    cv2.imshow("Robot", img)

# 讀取影像
image = cv2.imread('path_image.jpg')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (5, 5), 0)
edges = cv2.Canny(blur, 50, 150)

# 霍夫變換檢測直線
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=10)

# 繪製檢測到的直線
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

# 顯示影像
cv2.imshow('Detected Lines', image)

# 等待用戶按鍵
cv2.waitKey(0)

# 關閉所有顯示窗口
cv2.destroyAllWindows()
ep_camera.stop_video_stream()
ep_robot.close()
