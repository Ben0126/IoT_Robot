import cv2
from robomaster import robot, vision
import numpy as np
import time

# 定義一個類來儲存標記訊息
class MarkerInfo:
    def __init__(self, x, y, w, h, info):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.info = info

    @property   # 計算左上角的座標點
    def pt1(self):
        return int((self.x - self.w / 2) * 1280), int((self.y - self.h / 2) * 720)

    @property   # 計算右下角的座標點
    def pt2(self):
        return int((self.x + self.w / 2) * 1280), int((self.y + self.h / 2) * 720)

    @property   # 計算中心點
    def center(self):
        return int(self.x * 1280), int(self.y * 720)

# 定義一個類來儲存點的信息
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

markers = []
def on_detect_marker(marker_info):
    number = len(marker_info)
    for i in range(0, number):
        x, y, w, h, info = marker_info[i]
        markers.append(MarkerInfo(x, y, w, h, info))

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

def configure_robot(ep_robot):
    return (ep_robot.vision, ep_robot.camera, ep_robot.chassis, ep_robot.gripper, ep_robot.sensor, ep_robot.gimbal)

def track_line(ep_robot, mode):
    ep_vision, ep_camera, ep_chassis, ep_gripper, ep_sensor, ep_gimbal = ep_robot.vision, ep_robot.camera, ep_robot.chassis, ep_robot.gripper, ep_robot.sensor, ep_robot.gimbal
    ep_camera.start_video_stream(display=False)
    print("in track_line")

    try:
        if mode == "cargo-marker":
            print("cargo-marker")
            configure_marker_execute(ep_robot, ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor, "cargo")
        # elif mode == "arm-distance":
        #     print("arm-distanc")
        #     configure_distance_execute(ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor)
        # elif mode == "arm-marker":
        #     print("arm-marker")
        #     configure_marker_execute(ep_robot, ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor, "arm")

    except Exception as e:
        print("An error occurred: %s", e)
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
        time.sleep(1)
    finally:
        ep_vision.unsub_detect_info(name="line")
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        ep_robot.close()

def configure_marker_execute(ep_robot, ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor, mode):
    pid = PIDController(Kp=27, Ki=0, Kd=2.5)
    x_val = 0.2
    if mode == "cargo":
        while True:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            frame_width = img.shape[1]

            ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
            print("line 11")
            for j in range(0, len(markers)):
                cv2.rectangle(img, markers[j].pt1, markers[j].pt2, (255, 255, 255))
            print("line 13")
            if markers:
                stop_and_reposition(ep_chassis)
                ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
                time.sleep(1)
                # ep_robot.close()
                print("line 17")
                break
            ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)

            print("CARGO")
            process_lines_cargo(ep_chassis, img, pid, x_val, frame_width, line_list)

            # cv2.imshow("Line", img)
            if cv2.waitKey(1) & 0xFF == ord(' '):
                stop_and_reposition(ep_chassis)
                break


def process_lines_cargo(ep_chassis, img, pid, x_val, frame_width, line_list):
    if line_list:
        recent_points = line_list[:-6]
        avg_x = int(np.mean([p.pt[0] for p in recent_points]))
        avg_y = int(np.mean([p.pt[1] for p in recent_points]))

        setpoint = frame_width // 2
        control_signal = pid.compute(setpoint, avg_x) / 100
        ep_chassis.drive_speed(x=x_val, y=0, z=control_signal)

        for point in line_list:
            cv2.circle(img, point.pt, 3, point.color, -1)
        cv2.circle(img, (avg_x, avg_y), 5, (0, 255, 0), -1)


def stop_and_reposition(ep_chassis):
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
    time.sleep(1)

if __name__ == "__main__":
    try:
        robot1 = robot.Robot()
        robot1.initialize(conn_type="sta", sn="3JKDH6C001462K")

        track_line(robot1, "cargo-marker")
        print("track line")
        task = markers[0].info if markers else None
        robot1.close()
        print("task:", task)
        time.sleep(1)
        print("The End")

    except KeyboardInterrupt:
        stop_and_reposition(robot1)
        # stop_and_reposition(robot2)
        print("Emergency stop!")

    finally:
        robot1.close()
        # robot2.close()
        print("All robots closed")