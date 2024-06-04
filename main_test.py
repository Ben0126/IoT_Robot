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
    global markers
    markers = [MarkerInfo(x, y, w, h, info) for x, y, w, h, info in marker_info]
    print("Detected markers:", [m.text for m in markers])

    # number = len(marker_info)
    # markers.clear()
    # for i in range(0, number):
    #     x, y, w, h, info = marker_info[i]
    #     markers.append(MarkerInfo(x, y, w, h, info))
    #     print("marker:{0} x:{1}, y:{2}, w:{3}, h:{4}".format(info, x, y, w, h))

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
    global line_list
    line_list = [PointInfo(x, y, ceta, c) for x, y, ceta, c in line_info[1:]]

    # number = len(line_info)
    # line_list.clear()  # 清空现有的线的点
    # line_type = line_info[0]  # 线的类型信息
    # for i in range(1, number):
    #     x, y, ceta, c = line_info[i]
    #     line_list.append(PointInfo(x, y, ceta, c))  # 将新的点信息添加到line列表中

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

def main_car1():
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
    print("Gripper opened")

    pid = PIDController(Kp=26, Ki=0, Kd=2.5)

    try:
        while True:
            ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            frame_width = img.shape[1]  # 获取图像宽度

            for marker in markers:
                cv2.rectangle(img, marker.pt1, marker.pt2, (255, 255, 255))
                cv2.putText(img, marker.text, marker.center, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

            if markers:
                ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
                time.sleep(1)
                # 迴轉 180度
                ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
                time.sleep(0.5)
                # 后退 0.2米
                ep_chassis.move(x=-0.2, y=0, z=0, xy_speed=0.7).wait_for_completed()
                break

            # for j in range(0, len(markers)):
            #     cv2.rectangle(img, markers[j].pt1, markers[j].pt2, (255, 255, 255))
            #     cv2.putText(img, markers[j].text, markers[j].center, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255),
            #                 3)
            #
            # if len(markers) > 0:
            #     ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
            #     time.sleep(1)
            #     break

            ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
            if line_list:
                # 取最后10个检测到的点的平均数值
                recent_points = line_list[-10:]
                avg_x = int(np.mean([p.pt[0] for p in recent_points]))
                avg_y = int(np.mean([p.pt[1] for p in recent_points]))

                setpoint = frame_width // 2  # 设定点为图像中心

                # 计算PID控制量
                control_signal = pid.compute(setpoint, avg_x) / 100
                ep_chassis.drive_speed(x=0.5, y=0, z=control_signal)

                # 在图像上绘制检测到的点
                for point in line_list:
                    cv2.circle(img, point.pt, 3, point.color, -1)

                # 在图像上绘制平均点
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

def main_car2(markers):
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKDH5D0017578")

    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_gripper = ep_robot.gripper
    ep_servo = ep_robot.servo

    x_val = 0.5
    y_val = 0.6
    z_val = 90

    ep_camera.start_video_stream(display=False)

    # 根據 markers 資訊決定第二台車的任務
    for marker in markers:
        print(f"Marker text: {marker.text}")
        task = marker.text

    ep_gripper.open(power=10)
    time.sleep(2)
    ep_gripper.pause()
    time.sleep(1)
    print("Gripper opened")

    if task == "1":
        # 左转 90度
        ep_chassis.move(x=0, y=0, z=z_val, z_speed=45).wait_for_completed()
        time.sleep(0.5)
        print("Task 1")

    elif task =="2":
        # 待在原地
        ep_chassis.move(x=0, y=0, z=0, xy_speed=0).wait_for_completed()
        time.sleep(0.5)
        print("Task 2")

    elif task =="3":
        # 右转 90度
        ep_chassis.move(x=0, y=0, z=-z_val, z_speed=45).wait_for_completed()
        time.sleep(0.5)
        print("Task 3")

    else:
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
        time.sleep(1)
        print("STOP")

    # 放下夾子
    ep_servo.moveto(index=1, angle=-10).wait_for_completed()
    ep_servo.moveto(index=2, angle=-35).wait_for_completed()

    # 前進 0.5m
    ep_chassis.move(x=x_val, y=0, z=0, xy_speed=0.7).wait_for_completed()

    # 夾起物品
    ep_gripper.close(power=10)
    time.sleep(1.5)
    ep_gripper.pause()
    time.sleep(1)
    print("Gripper closed")
    ep_servo.moveto(index=2, angle=-95).wait_for_completed()
    ep_servo.moveto(index=1, angle=80).wait_for_completed()
    time.sleep(2)
    print("Finish")

    # 迴轉 回到起點
    ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
    time.sleep(0.5)
    ep_chassis.move(x=x_val, y=0, z=0, xy_speed=0.7).wait_for_completed()

    if task == "1":
        # 右转 90度
        ep_chassis.move(x=0, y=0, z=-z_val, z_speed=45).wait_for_completed()
        time.sleep(0.5)

    elif task =="2":
        # 待在原地
        ep_chassis.move(x=0, y=0, z=0, xy_speed=0).wait_for_completed()
        time.sleep(0.5)

    elif task =="3":
        # 左转 90度
        ep_chassis.move(x=0, y=0, z=z_val, z_speed=45).wait_for_completed()
        time.sleep(0.5)

    else:
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
        time.sleep(1)
        print("STOP")


    ep_chassis.move(x=0.2, y=0, z=0, xy_speed=0.7).wait_for_completed()
    time.sleep(0.5)
    ep_servo.moveto(index=1, angle=-10).wait_for_completed()
    ep_servo.moveto(index=2, angle=-55).wait_for_completed()
    ep_gripper.open(power=10)
    time.sleep(2)
    ep_gripper.pause()
    time.sleep(1)
    ep_chassis.move(x=-0.2, y=0, z=0, xy_speed=0.7).wait_for_completed()
    time.sleep(0.5)
    print("The End from Arm Robot")

    # 确保在异常或退出时正确释放资源
    ep_vision.unsub_detect_info(name="line")
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()


if __name__ == '__main__':
    start = time.time()
    main_car1()
    main_car2(markers)
    main_car1()
    end = time.time()
    print(f"Execution time: {end - start:.2f} seconds")