import cv2
from robomaster import robot, vision
import numpy as np
import time

class MarkerInfo:
    def __init__(self, x, y, w, h, info):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.info = info

    @property
    def pt1(self):
        return int((self.x - self.w / 2) * 1280), int((self.y - self.h / 2) * 720)

    @property
    def pt2(self):
        return int((self.x + self.w / 2) * 1280), int((self.y + self.h / 2) * 720)

    @property
    def center(self):
        return int(self.x * 1280), int(self.y * 720)

markers = []

def on_detect_marker(marker_info):
    global markers
    markers = [MarkerInfo(x, y, w, h, info) for x, y, w, h, info in marker_info]
    print("Detected markers:", [m.info for m in markers])

class PointInfo:
    def __init__(self, x, y, theta, c):
        self.x = x
        self.y = y
        self.theta = theta
        self.c = c

    @property
    def pt(self):
        return int(self.x * 1280), int(self.y * 720)

    @property
    def color(self):
        return 255, 255, 255

line_list = []

def on_detect_line(line_info):
    global line_list
    line_list = [PointInfo(x, y, ceta, c) for x, y, ceta, c in line_info[1:]]

# 初始化一個變數來保存距離數據
distance_data = None
# 回調函數來處理距離數據
def sub_data_distance(sub_info):
    global distance_data
    distance = sub_info
    distance_data = distance[0]
    print("tof1:{0}".format(distance_data))

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

def track_line(ep_robot, mode, markers):
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_gripper = ep_robot.gripper
    ep_sensor = ep_robot.sensor
    ep_gimbal = ep_robot.gimbal

    ep_camera.start_video_stream(display=False)


    ep_gripper.open(power=10)
    time.sleep(2)
    ep_gripper.pause()
    time.sleep(1)
    print("Gripper opened")

    # mode 1: cargo-marker
    if mode == 1:
        Kp = 33
        Ki = 0
        Kd = 0.5
        x_val = 0.5

    # mode 2: arm-distance
    elif mode == 2:
        Kp = 33
        Ki = 0
        Kd = 0.5
        x_val = 0.1

    # mode 3: arm-marker
    elif mode == 3:
        Kp = 33
        Ki = 0
        Kd = 0.5
        x_val = 0.5

    pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd)

    try:
        while True:
            if mode == 1:
                ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
                img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
                frame_width = img.shape[1]  # 获取图像宽度

                for marker in markers:
                    cv2.rectangle(img, marker.pt1, marker.pt2, (255, 255, 255))
                    cv2.putText(img, marker.text, marker.center, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

                if markers:
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
                    time.sleep(1)
                    # # 迴轉 180度
                    # ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
                    # time.sleep(0.5)
                    # # 后退 0.2米
                    # ep_chassis.move(x=-0.2, y=0, z=0, xy_speed=0.7).wait_for_completed()
                    break

                ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
                if line_list:
                    # 取最后10个检测到的点的平均数值
                    recent_points = line_list[-10:]
                    avg_x = int(np.mean([p.pt[0] for p in recent_points]))
                    avg_y = int(np.mean([p.pt[1] for p in recent_points]))

                    setpoint = frame_width // 2  # 设定点为图像中心

                    # 计算PID控制量
                    control_signal = pid.compute(setpoint, avg_x) / 100
                    ep_chassis.drive_speed(x=x_val, y=0, z=control_signal)

                    # 在图像上绘制检测到的点
                    for point in line_list:
                        cv2.circle(img, point.pt, 3, point.color, -1)

                    # 在图像上绘制平均点
                    cv2.circle(img, (avg_x, avg_y), 5, (0, 255, 0), -1)

            elif mode == 2:
                ep_sensor.sub_distance(freq=5, callback=sub_data_distance)
                ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
                ep_gimbal.moveto(pitch=-50, yaw=0).wait_for_completed()
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
                    ep_chassis.drive_speed(x=x_val, y=0, z=control_signal)

                    # 在图像上绘制检测到的点
                    for point in line_list:
                        cv2.circle(img, point.pt, 3, point.color, -1)

                    # 在图像上绘制平均点
                    cv2.circle(img, (avg_x, avg_y), 5, (0, 255, 0), -1)

                if distance_data is not None and distance_data <= 50:
                    print("find")
                    ep_chassis.drive_speed(x=0, y=0, z=0)
                    time.sleep(1)
                    ep_chassis.move(x=0.05, y=0, z=0, xy_speed=0.01).wait_for_completed()
                    break

            elif mode == 3:
                ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
                img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
                frame_width = img.shape[1]  # 获取图像宽度

                for marker in markers:
                    cv2.rectangle(img, marker.pt1, marker.pt2, (255, 255, 255))
                    cv2.putText(img, marker.text, marker.center, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

                if markers:
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
                    time.sleep(1)
                    # # 迴轉 180度
                    # ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
                    # time.sleep(0.5)
                    # # 后退 0.2米
                    # ep_chassis.move(x=-0.2, y=0, z=0, xy_speed=0.7).wait_for_completed()
                    break

                ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
                if line_list:
                    # 取最后10个检测到的点的平均数值
                    recent_points = line_list[-10:]
                    avg_x = int(np.mean([p.pt[0] for p in recent_points]))
                    avg_y = int(np.mean([p.pt[1] for p in recent_points]))

                    setpoint = frame_width // 2  # 设定点为图像中心

                    # 计算PID控制量
                    control_signal = pid.compute(setpoint, avg_x) / 100

                    ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=control_signal)
                    ep_chassis.drive_speed(x=x_val, y=0, z=control_signal)

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
        ep_vision.unsub_detect_info(name="line")
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        ep_robot.close()


if __name__ == '__main__':
    try:
        # 初始化机器人
        # Cargo: 3JKDH6C001462K
        robot1 = robot.Robot()
        robot1.initialize(conn_type="sta", sn="3JKDH6C001462K")
        ep1_vision = robot1.vision
        ep1_camera = robot1.camera
        ep1_chassis = robot1.chassis
        ep1_gripper = robot1.gripper

        # Arm: 3JKDH5D0017578
        robot2 = robot.Robot()
        robot2.initialize(conn_type="sta", sn="3JKDH5D0017578")
        ep2_vision = robot2.vision
        ep2_camera = robot2.camera
        ep2_chassis = robot2.chassis
        ep2_gripper = robot2.gripper
        ep2_servo = robot2.servo
        ep2_gimbal = robot2.gimbal

        # 运行主要功能
        track_line(robot1, 1)

        # 迴轉 180度
        ep1_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
        time.sleep(0.5)

        # 根據 markers 資訊決定第二台車的任務
        try:
            for marker in markers:
                print(f"Marker text: {marker.text}")
                task = marker.text
        except:
            print("No task")

        x_val = 0.5
        y_val = 0.6
        z_val = 90

        if task == "1":
            ep2_chassis.move(x=0, y=0, z=z_val, z_speed=45).wait_for_completed()
            time.sleep(0.5)
            print("Task 1")

        elif task == "2":
            ep2_chassis.move(x=0, y=0, z=0, xy_speed=0).wait_for_completed()
            time.sleep(0.5)
            print("Task 2")

        elif task == "3":
            ep2_chassis.move(x=0, y=0, z=-z_val, z_speed=45).wait_for_completed()
            time.sleep(0.5)
            print("Task 3")

        else:
            ep2_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
            time.sleep(1)
            print("STOP")

        ep2_servo.moveto(index=1, angle=-10).wait_for_completed()
        ep2_servo.moveto(index=2, angle=-35).wait_for_completed()
        ep2_gimbal.moveto(pitch=-50, yaw=0).wait_for_completed()
        time.sleep(0.5)

        track_line(robot2, 2)

        # 夾起物品
        print("catch")
        time.sleep(1)
        ep2_gripper.close(power=10)
        time.sleep(1.5)
        ep2_gripper.pause()
        time.sleep(0.5)
        print("Gripper closed")
        ep2_servo.moveto(index=2, angle=-95).wait_for_completed()
        ep2_servo.moveto(index=1, angle=80).wait_for_completed()
        time.sleep(2)
        print("Finish")

        # 迴轉 回到起點
        ep2_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
        ep2_gimbal.moveto(pitch=-50, yaw=0, pitch_speed=100, yaw_speed=100).wait_for_completed()

        track_line(robot2,3)

        if task == "1":
            ep2_chassis.move(x=0, y=0, z=-z_val, z_speed=45).wait_for_completed()
            time.sleep(0.5)

        elif task == "2":
            ep2_chassis.move(x=0, y=0, z=0, xy_speed=0).wait_for_completed()
            time.sleep(0.5)

        elif task == "3":
            ep2_chassis.move(x=0, y=0, z=z_val, z_speed=45).wait_for_completed()
            time.sleep(0.5)

        else:
            ep2_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
            time.sleep(1)
            print("STOP")

        ep1_chassis.move(x=-0.2, y=0, z=0, xy_speed=0.7).wait_for_completed()
        time.sleep(0.5)

        # 放到 Cargo
        ep2_servo.moveto(index=1, angle=-10).wait_for_completed()
        ep2_servo.moveto(index=2, angle=-55).wait_for_completed()
        ep2_gripper.open(power=10)
        time.sleep(2)
        ep2_gripper.pause()
        time.sleep(1)
        print("The End from Arm Robot")
        time.sleep(5)
        track_line(robot1,1)
        print("The End")

    except KeyboardInterrupt:
        print("Emergency stop!")

    finally:
        # 确保所有机器人都关闭连接
        robot1.close()
        robot2.close()
        print("All robots closed")