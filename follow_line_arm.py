import cv2
from robomaster import robot, vision
import numpy as np
import time


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


def sub_data_handler(sub_info):
    distance = sub_info
    print("tof1:{0}".format(distance[0]))


def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKDH5D0017578")

    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_sensor = ep_robot.sensor

    ep_camera.start_video_stream(display=False)
    ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
    ep_sensor.sub_distance(freq=5, callback=sub_data_handler)
    # ep_gimbal.sub_angle(freq=5, callback=sub_data_handler)
    pid = PIDController(Kp=33, Ki=0, Kd=1)
    ep_gimbal.moveto(pitch=-50, yaw=0).wait_for_completed()
    time.sleep(2)
    try:
        while True:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            frame_width = img.shape[1]  # 获取图像宽度

            if line_list:
                # 取最后10个检测到的点的平均数值
                recent_points = line_list[:-6]
                avg_x = int(np.mean([p.pt[0] for p in recent_points]))
                avg_y = int(np.mean([p.pt[1] for p in recent_points]))

                setpoint = frame_width // 2  # 设定点为图像中心
                print('current_point', avg_x)

                # 计算PID控制量
                control_signal = pid.compute(setpoint, avg_x)
                control_signal /= 100
                print('control_signal', control_signal)

                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=control_signal)
                ep_chassis.drive_speed(x=1, y=0, z=control_signal)

                # 在图像上绘制检测到的点
                for point in line_list:
                    cv2.circle(img, point.pt, 3, point.color, -1)

                # 在图像上绘制平均点
                cv2.circle(img, (avg_x, avg_y), 5, (0, 255, 0), -1)

            cv2.imshow("Line", img)
            if cv2.waitKey(1) & 0xFF == ord(' '):
                ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
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