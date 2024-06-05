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

def sub_distance_data(sub_info):
    distance = sub_info
    print("tof1:{0}".format(distance[0]))

def main_car1(ep_robot):
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
            frame_width = img.shape[1]

            for marker in markers:
                cv2.rectangle(img, marker.pt1, marker.pt2, (255, 255, 255))
                cv2.putText(img, marker.info, marker.center, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

            if markers:
                ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
                time.sleep(1)
                ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
                time.sleep(0.5)
                ep_chassis.move(x=-0.2, y=0, z=0, xy_speed=0.7).wait_for_completed()
                break

            ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
            if line_list:
                recent_points = line_list[-10:]
                avg_x = int(np.mean([p.pt[0] for p in recent_points]))
                avg_y = int(np.mean([p.pt[1] for p in recent_points]))

                setpoint = frame_width // 2
                control_signal = pid.compute(setpoint, avg_x) / 100
                ep_chassis.drive_speed(x=0.5, y=0, z=control_signal)

                for point in line_list:
                    cv2.circle(img, point.pt, 3, point.color, -1)

                cv2.circle(img, (avg_x, avg_y), 5, (0, 255, 0), -1)

            cv2.imshow("Line", img)

            if cv2.waitKey(1) & 0xFF == ord(' '):
                print("Emergency stop!")
                break

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        ep_vision.unsub_detect_info(name="line")
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()

def main_car2(ep_robot, markers):
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_gripper = ep_robot.gripper
    ep_servo = ep_robot.servo

    x_val = 0.5
    y_val = 0.6
    z_val = 90

    ep_camera.start_video_stream(display=False)

    for marker in markers:
        print(f"Marker text: {marker.info}")
        task = marker.info

    ep_gripper.open(power=10)
    time.sleep(2)
    ep_gripper.pause()
    time.sleep(1)
    print("Gripper opened")

    try:
        if task == "1":
            ep_chassis.move(x=0, y=0, z=z_val, z_speed=45).wait_for_completed()
            time.sleep(0.5)
            print("Task 1")

        elif task == "2":
            ep_chassis.move(x=0, y=0, z=0, xy_speed=0).wait_for_completed()
            time.sleep(0.5)
            print("Task 2")

        elif task == "3":
            ep_chassis.move(x=0, y=0, z=-z_val, z_speed=45).wait_for_completed()
            time.sleep(0.5)
            print("Task 3")

        else:
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
            time.sleep(1)
            print("STOP")

        ep_servo.moveto(index=1, angle=-10).wait_for_completed()
        ep_servo.moveto(index=2, angle=-35).wait_for_completed()

        ep_chassis.move(x=x_val, y=0, z=0, xy_speed=0.7).wait_for_completed()

        ep_gripper.close(power=10)
        time.sleep(1.5)
        ep_gripper.pause()
        time.sleep(1)
        print("Gripper closed")
        ep_servo.moveto(index=2, angle=-95).wait_for_completed()
        ep_servo.moveto(index=1, angle=80).wait_for_completed()
        time.sleep(2)
        print("Finish")

        ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
        time.sleep(0.5)
        ep_chassis.move(x=x_val, y=0, z=0, xy_speed=0.7).wait_for_completed()

        if task == "1":
            ep_chassis.move(x=0, y=0, z=-z_val, z_speed=45).wait_for_completed()
            time.sleep(0.5)

        elif task == "2":
            ep_chassis.move(x=0, y=0, z=0, xy_speed=0).wait_for_completed()
            time.sleep(0.5)

        elif task == "3":
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

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        ep_vision.unsub_detect_info(name="line")
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()

if __name__ == '__main__':
    try:
        # 初始化机器人
        robot1 = robot.Robot()
        robot2 = robot.Robot()

        # 3JKDH5D0017578 : Arm; 3JKDH6C001462K : Cargo
        robot1.initialize(conn_type="sta", sn="3JKDH6C001462K")
        robot2.initialize(conn_type="sta", sn="3JKDH5D0017578")


        # 运行主要功能
        main_car1(robot1)
        main_car2(robot2, markers)
        main_car1(robot1)

    except KeyboardInterrupt:
        print("Emergency stop!")

    finally:
        # 确保所有机器人都关闭连接
        robot1.close()
        robot2.close()
        print("All robots closed")
