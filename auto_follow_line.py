import cv2
from robomaster import robot, vision
import numpy as np
import time


class MarkerInfo:
    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

    @property
    def pt1(self):
        return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)

    @property
    def text(self):
        return self._info

markers = []

def on_detect_marker(marker_info):
    number = len(marker_info)
    markers.clear()
    for i in range(0, number):
        x, y, w, h, info = marker_info[i]
        markers.append(MarkerInfo(x, y, w, h, info))
        print("marker:{0} x:{1}, y:{2}, w:{3}, h:{4}".format(info, x, y, w, h))

class PointInfo:
    def __init__(self, x, y, theta, c):
        self._x = x  # 点的x坐标
        self._y = y  # 点的y坐标
        self._theta = theta  # 点的角度信息
        self._c = c  # 点的其他信息

    @property
    def pt(self):
        # 将x, y坐标转换为图像的像素坐标
        return int(self._x * 1280), int(self._y * 720)

    @property
    def color(self):
        # 返回用于绘制点的颜色 (白色)
        return 255, 255, 255

line_list = []  # 存储检测到的线的点的列表

def on_detect_line(line_info):
    number = len(line_info)
    line_list.clear()  # 清空现有的线的点
    line_type = line_info[0]  # 线的类型信息
    for i in range(1, number):
        x, y, ceta, c = line_info[i]
        line_list.append(PointInfo(x, y, ceta, c))  # 将新的点信息添加到line列表中

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, current_value):
        error = current_value - setpoint
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKDH6C001462K")

    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_gripper = ep_robot.gripper

    ep_camera.start_video_stream(display=False)

    ep_gripper.open(power=10)
    time.sleep(2)
    ep_gripper.pause()
    time.sleep(1)
    print("open")

    pid = PIDController(Kp=26, Ki=0, Kd=2.5)

    try:
        while True:
            ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            frame_width = img.shape[1]  # 獲取圖像寬度

            for j in range(0, len(markers)):
                cv2.rectangle(img, markers[j].pt1, markers[j].pt2, (255, 255, 255))
                cv2.putText(img, markers[j].text, markers[j].center, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255),
                            3)

            if len(markers) > 0:
                ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
                time.sleep(1)
                break

            ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
            if line_list:
                # 取最後10個檢測到的點的平均數值
                recent_points = line_list[-10:]
                avg_x = int(np.mean([p.pt[0] for p in recent_points]))
                avg_y = int(np.mean([p.pt[1] for p in recent_points]))

                setpoint = frame_width // 2  # 設定點為圖象中心

                # 計算PID控制量
                control_signal = pid.compute(setpoint, avg_x)
                control_signal /= 100

                ep_chassis.drive_speed(x=0.5, y=0, z=control_signal)

                # 圖像上繪製檢測到的點
                for point in line_list:
                    cv2.circle(img, point.pt, 3, point.color, -1)

                # 在圖像上繪製平均點
                cv2.circle(img, (avg_x, avg_y), 5, (0, 255, 0), -1)

            cv2.imshow("Line", img)


            if cv2.waitKey(1) & 0xFF == ord(' '):
                ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
                time.sleep(1)
                break

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # 确保在异常或退出时正确释放资源
        ep_vision.unsub_detect_info(name="line")
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        ep_robot.close()

if __name__ == '__main__':
    start = time.time()
    main()
    end = time.time()
    print(format(end - start))