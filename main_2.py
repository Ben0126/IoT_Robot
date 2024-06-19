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

# 初始化一個變數來保存距離數據
robot2_distance_data = None
# 回調函數來處理距離數據
def robot2_sub_data_handler(sub_info):
    global robot2_distance_data
    robot2_distance = sub_info
    robot2_distance_data = robot2_distance[0]
    print("tof1:{0}".format(robot2_distance_data))

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
        # if mode == "cargo-marker":
        #     print("cargo-marker")
        #     configure_marker_execute(ep_robot, ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor, "cargo")
        if mode == "arm-distance":
            print("arm-distanc")
            configure_distance_execute(ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor)
        elif mode == "arm-marker":
            print("arm-marker")
            configure_marker_execute(ep_robot, ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor, "arm")

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
    # if mode == "cargo":
    #     while True:
    #         img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
    #         frame_width = img.shape[1]
    #
    #         ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
    #         print("line 11")
    #         for j in range(0, len(markers)):
    #             cv2.rectangle(img, markers[j].pt1, markers[j].pt2, (255, 255, 255))
    #         print("line 13")
    #         if markers:
    #             stop_and_reposition(ep_chassis)
    #             ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
    #             time.sleep(1)
    #             # ep_robot.close()
    #             print("line 17")
    #             break
    #         ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
    #
    #         print("CARGO")
    #         process_lines_cargo(ep_chassis, img, pid, x_val, frame_width, line_list)
    #
    #         # cv2.imshow("Line", img)
    #         if cv2.waitKey(1) & 0xFF == ord(' '):
    #             stop_and_reposition(ep_chassis)
    #             break

    if mode == "arm":
        ep_gimbal.moveto(pitch=-50, yaw=0, pitch_speed=100, yaw_speed=100).wait_for_completed()
        print("head down")
        time.sleep(5)
        while True:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            frame_width = img.shape[1]

            ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
            print("line 110")
            for j in range(0, len(markers)):
                cv2.rectangle(img, markers[j].pt1, markers[j].pt2, (255, 255, 255))
            print("line 113")

            if markers:
                stop_and_reposition(ep_chassis)
                # ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
                time.sleep(1)
                print("catch photo")
                time.sleep(1)
                break

            ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
            print("ARM")
            process_lines_arm(ep_chassis, ep_gimbal, img, pid, x_val, frame_width, line_list)
            print("line 142")

            cv2.imshow("Line", img)
            if cv2.waitKey(1) & 0xFF == ord(' '):
                stop_and_reposition(ep_chassis)
                break

def configure_distance_execute(ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor):
    pid = PIDController(Kp=27, Ki=0, Kd=2.5)
    x_val = 0.1
    ep_sensor.sub_distance(freq=5, callback=robot2_sub_data_handler)
    ep_gimbal.moveto(pitch=-50, yaw=0, pitch_speed=100, yaw_speed=100).wait_for_completed()
    print("head down")
    time.sleep(5)

    while True:
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.2)
        frame_width = img.shape[1]

        ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
        process_lines_arm(ep_chassis, ep_gimbal, img, pid, x_val, frame_width, line_list)
        print("follow line")

        print(robot2_distance_data)
        if robot2_distance_data is not None and robot2_distance_data <= 50:
            ep_chassis.drive_speed(x=0, y=0, z=0)
            print("FUCK")
            ep_chassis.move(x=0.05, y=0, z=0, xy_speed=0.01).wait_for_completed()
            stop_and_reposition(ep_chassis)
            break

        cv2.imshow("Line", img)
        if cv2.waitKey(1) & 0xFF == ord(' '):
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
            time.sleep(1)
            # stop_and_reposition(ep_chassis)
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

def process_lines_arm(ep_chassis, ep_gimbal, img, pid, x_val, frame_width, line_list):
    print("in process_lines_arm")
    if line_list:
        print("in 201")
        recent_points = line_list[:-6]
        avg_x = int(np.mean([p.pt[0] for p in recent_points]))
        avg_y = int(np.mean([p.pt[1] for p in recent_points]))
        print("in 205")
        setpoint = frame_width // 2
        print("in 206")
        control_signal = pid.compute(setpoint, avg_x) / 100
        print("in 207")
        ep_chassis.drive_speed(x=x_val, y=0, z=control_signal)
        print("in 208")
        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=control_signal)

        print("in 210")
        for point in line_list:
            cv2.circle(img, point.pt, 3, point.color, -1)
        cv2.circle(img, (avg_x, avg_y), 5, (0, 255, 0), -1)
        print("in 214")

def stop_and_reposition(ep_chassis):
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
    time.sleep(1)

def execute_task(robot, task):
    ep_chassis, ep_servo, ep_gimbal, ep_gripper = robot.chassis, robot.servo, robot.gimbal, robot.gripper

    if task == "1":
        ep_chassis.move(x=0, y=0, z=90, z_speed=45).wait_for_completed()
    elif task == "2":
        ep_chassis.move(x=0, y=0, z=0, xy_speed=0).wait_for_completed()
    elif task == "3":
        ep_chassis.move(x=0, y=0, z=-90, z_speed=45).wait_for_completed()
    else:
        stop_and_reposition(ep_chassis)
        print("No get task")
    time.sleep(0.5)

    ep_servo.moveto(index=1, angle=-10).wait_for_completed()
    ep_servo.moveto(index=2, angle=-35).wait_for_completed()
    ep_gimbal.moveto(pitch=-50, yaw=0).wait_for_completed()
    ep_gripper.open(power=10)
    time.sleep(0.5)

def catch_and_return(robot):
    ep_chassis, ep_gripper, ep_servo, ep_gimbal = robot.chassis, robot.gripper, robot.servo, robot.gimbal
    # ep_gripper.open(power=10)
    # time.sleep(1)
    ep_chassis.move(x=0.03, y=0, z=0, xy_speed=0.1).wait_for_completed()
    print("catch")
    ep_gripper.close(power=10)
    print("line 227")
    time.sleep(2)
    ep_gripper.pause()
    print("line 230")
    time.sleep(1)
    print("Gripper closed")
    ep_servo.moveto(index=2, angle=-95).wait_for_completed()
    ep_servo.moveto(index=1, angle=80).wait_for_completed()
    print("Finish")
    time.sleep(2)
    ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
    ep_gimbal.moveto(pitch=-50, yaw=0, pitch_speed=100, yaw_speed=100).wait_for_completed()
    time.sleep(0.5)

def execute_task_back(robot, task):
    ep_chassis, ep_gripper, ep_servo, ep_gimbal = robot.chassis, robot.gripper, robot.servo, robot.gimbal

    if task == "1":
        print("1")
        ep_chassis.move(x=0, y=0, z=-90, z_speed=45).wait_for_completed()

    if task == "2":
        print("2")
        ep_chassis.move(x=0, y=0, z=0, xy_speed=0).wait_for_completed()

    if task == "3":
        print("3")
        ep_chassis.move(x=0, y=0, z=90, z_speed=45).wait_for_completed()

    time.sleep(0.5)
    print("put")
    ep_chassis.move(x=0.2, y=0, z=0, xy_speed=0.1).wait_for_completed()
    time.sleep(0.5)
    ep_servo.moveto(index=1, angle=-10).wait_for_completed()
    ep_servo.moveto(index=2, angle=-55).wait_for_completed()
    ep_gripper.open(power=10)
    time.sleep(1)
    ep_gripper.pause()
    time.sleep(1)
    ep_chassis.move(x=-0.1, y=0, z=0, xy_speed=0.1).wait_for_completed()
    ep_servo.moveto(index=2, angle=-95).wait_for_completed()
    ep_servo.moveto(index=1, angle=80).wait_for_completed()
    time.sleep(0.5)
    ep_chassis.move(x=-0.5, y=0, z=0, xy_speed=0.7).wait_for_completed()
    ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
    print("The End from Arm Robot")


# robot1_distance_data = None
# def robot1_sub_data_handler(sub_info):
#     global robot1_distance_data
#     robot1_distance = sub_info
#     robot1_distance_data = robot1_distance[0]
#     print("tof1:{0}".format(robot1_distance_data))
# def get_box(robot):
#     ep_sensor, ep_chassis = robot.sensor, robot.chassis
#     ep_sensor.sub_distance(freq=5, callback=robot1_sub_data_handler)
#     while True:
#         ep_chassis.drive_speed(x=-0.05, y=0, z=0)
#         if robot1_distance_data is not None and robot1_distance_data <= 50:
#             # ep_chassis.drive_speed(x=0, y=0, z=0)
#             stop_and_reposition(ep_chassis)
#             print("arrive robot2")
#             time.sleep(1)
#             ep_chassis.move(x=0, y=-0.18, z=0, xy_speed=1).wait_for_completed()
#             print("arrive")
#             break


if __name__ == "__main__":
    try:
        # robot1 = robot.Robot()
        # robot1.initialize(conn_type="sta", sn="3JKDH6C001462K")
        #
        # track_line(robot1, "cargo-marker")
        # print("track line")
        # task = markers[0].info if markers else None
        # robot1.close()
        # print("task:", task)
        # time.sleep(1)

        robot2 = robot.Robot()
        robot2.initialize(conn_type="sta", sn="3JKDH5D0017578")
        task = 2
        execute_task(robot2, task)
        print("go to specify place")
        track_line(robot2, "arm-distance")
        print("arrived specify place")
        robot2.close()
        time.sleep(1)

        robot2 = robot.Robot()
        robot2.initialize(conn_type="sta", sn="3JKDH5D0017578")
        time.sleep(1)
        print("re-connect")
        catch_and_return(robot2)
        robot2.close()

        robot2 = robot.Robot()
        robot2.initialize(conn_type="sta", sn="3JKDH5D0017578")
        print("catch_and_return")
        track_line(robot2, "arm-marker")
        print("arm_robot come back")
        robot2.close()

        # robot1 = robot.Robot()
        # robot1.initialize(conn_type="sta", sn="3JKDH6C001462K")
        # get_box(robot1)
        # robot1.close()

        # robot2 = robot.Robot()
        # robot2.initialize(conn_type="sta", sn="3JKDH5D0017578")
        # time.sleep(5)
        # execute_task_back(robot2, task)
        # robot2.close()

        # robot1 = robot.Robot()
        # robot1.initialize(conn_type="sta", sn="3JKDH6C001462K")
        # track_line(robot1, "cargo-marker")
        print("The End")

    except KeyboardInterrupt:
        # stop_and_reposition(robot1)
        stop_and_reposition(robot2)
        print("Emergency stop!")

    finally:
        # robot1.close()
        robot2.close()
        print("All robots closed")