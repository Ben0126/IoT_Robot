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

# def sub_data_distance(sub_info):
#     global distance_data
#     distance = sub_info
#     distance_data = distance[0]
#     print("tof1:{0}".format(distance_data))

distance_data = None
def sub_data_distance(sub_info):
    distance = sub_info
    distance_data = distance[0]
    print("tof1:{0}".format(distance[0]))

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
    ep_vision, ep_camera, ep_chassis, ep_gripper, ep_sensor, ep_gimbal = configure_robot(ep_robot)
    ep_camera.start_video_stream(display=False)
    print("line 81")

    try:
        print("line 84")
        if mode == "1":
            print("line 86")
            configure_and_execute_mode(ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor, "1")
            print("line 88")
        elif mode == "2":
            print("line 90")
            configure_and_execute_mode(ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor, "2")
            print("line 92")
        elif mode == "3":
            print("line 94")
            configure_and_execute_mode(ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor, "3")
            print("line 96")
        else:
            configure_and_execute_mode(ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor, "1")
            print("line 99")

        print("line 94")

    except Exception as e:
        print("91 An error occurred: %s", e)
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
        time.sleep(1)
    finally:
        ep_vision.unsub_detect_info(name="line")
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        ep_robot.close()

def configure_and_execute_mode(ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor, mode):
    print("line 112")
    pid = PIDController(Kp=27, Ki=0, Kd=2.5)
    x_val = 0.5 if mode != "2" else 0.1
    print("line 115")

    while True:
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        frame_width = img.shape[1]
        print("line 120")

        if mode == "1" or mode == "3":
            print("line 123")
            ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
            print("line 125")
            for j in range(0, len(markers)):
                print("line 135")
                cv2.rectangle(img, markers[j].pt1, markers[j].pt2, (255, 255, 255))
                print("line 140")

            if markers:
                print("marker break 145")
                stop_and_reposition(ep_chassis)
                # ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
                # time.sleep(1)
                ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
                break

            print("line 139")
            ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
            process_lines(ep_chassis, ep_gimbal, img, pid, x_val, frame_width, mode, line_list)
            print("line 142")

        elif mode == "2":
            print("line 146")
            ep_sensor.sub_distance(freq=5, callback=sub_data_distance)
            ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
            print("line 149")
            ep_gimbal.moveto(pitch=-50, yaw=0).wait_for_completed()
            print("line 151")
            process_lines(ep_chassis, ep_gimbal, img, pid, x_val, frame_width, mode, line_list)
            print("line 153")
            if distance_data is not None:
                print("distance_data : None")
            else:
                if distance_data <= 50:
                    print("distance_data < 50")
                    stop_and_reposition(ep_chassis)
                    ep_chassis.move(x=0.05, y=0, z=0, xy_speed=0.01).wait_for_completed()
                    break

        cv2.imshow("Line", img)
        if cv2.waitKey(1) & 0xFF == ord(' '):
            stop_and_reposition(ep_chassis)
            break

def process_lines(ep_chassis, ep_gimbal, img, pid, x_val, frame_width, mode, line_list):
    print("line 159")
    if line_list:
        recent_points = line_list[:-6]
        avg_x = int(np.mean([p.pt[0] for p in recent_points]))
        avg_y = int(np.mean([p.pt[1] for p in recent_points]))
        print("line 161")

        setpoint = frame_width // 2
        control_signal = pid.compute(setpoint, avg_x) / 100

        if mode == "2" or mode == "3":
            print("mode 2&3 track line")
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=control_signal)
            ep_chassis.drive_speed(x=x_val, y=0, z=control_signal)
        elif mode == "1":
            print("mode 1 track line")
            ep_chassis.drive_speed(x=x_val, y=0, z=control_signal)

        print("line 191")

        for point in line_list:
            cv2.circle(img, point.pt, 3, point.color, -1)
        cv2.circle(img, (avg_x, avg_y), 5, (0, 255, 0), -1)

def stop_and_reposition(ep_chassis):
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
    time.sleep(1)


def execute_task(robot, task):
    ep_chassis, ep_servo, ep_gimbal = robot.chassis, robot.servo, robot.gimbal
    tasks = {
        "1": lambda: ep_chassis.move(x=0, y=0, z=90, z_speed=45).wait_for_completed(),
        "2": lambda: ep_chassis.move(x=0, y=0, z=0, xy_speed=0).wait_for_completed(),
        "3": lambda: ep_chassis.move(x=0, y=0, z=-90, z_speed=45).wait_for_completed(),
    }
    tasks.get(task, lambda: stop_and_reposition(ep_chassis))()
    time.sleep(0.5)
    ep_servo.moveto(index=1, angle=-10).wait_for_completed()
    ep_servo.moveto(index=2, angle=-35).wait_for_completed()
    ep_gimbal.moveto(pitch=-50, yaw=0).wait_for_completed()
    time.sleep(0.5)

def catch_and_return(robot):
    ep_chassis, ep_gripper, ep_servo, ep_gimbal = robot.chassis, robot.gripper, robot.servo, robot.gimbal
    print("catch")
    ep_gripper.close(power=10)
    time.sleep(1.5)
    ep_gripper.pause()
    time.sleep(0.5)
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
    tasks = {
        "1": lambda: ep_chassis.move(x=0, y=0, z=-90, z_speed=45).wait_for_completed(),
        "2": lambda: ep_chassis.move(x=0, y=0, z=0, xy_speed=0).wait_for_completed(),
        "3": lambda: ep_chassis.move(x=0, y=0, z=90, z_speed=45).wait_for_completed(),
    }
    tasks.get(task, lambda: stop_and_reposition(ep_chassis))()
    time.sleep(0.5)
    ep_chassis.move(x=0.2, y=0, z=0, xy_speed=0.7).wait_for_completed()
    time.sleep(0.5)
    ep_servo.moveto(index=1, angle=-10).wait_for_completed()
    ep_servo.moveto(index=2, angle=-55).wait_for_completed()
    ep_gripper.open(power=10)
    time.sleep(2)
    ep_gripper.pause()
    time.sleep(1)
    ep_servo.moveto(index=2, angle=-95).wait_for_completed()
    ep_servo.moveto(index=1, angle=80).wait_for_completed()
    time.sleep(0.5)
    ep_chassis.move(x=-0.5, y=0, z=0, xy_speed=0.7).wait_for_completed()
    ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
    print("The End from Arm Robot")

if __name__ == "__main__":
    try:
        # robot1 = robot.Robot()
        # robot1.initialize(conn_type="sta", sn="3JKDH6C001462K")
        # ep1_vision, ep1_camera, ep1_chassis, ep1_gripper, ep1_sensor, ep1_gimbal = configure_robot(robot1)
        #
        # print("line 215")
        # track_line(robot1, "1")
        # task = markers[0].info if markers else None
        # markers.clear()
        # print("task:", task)
        # time.sleep(4)

        robot2 = robot.Robot()
        robot2.initialize(conn_type="sta", sn="3JKDH5D0017578")

        task = "2"
        execute_task(robot2, task)
        print("task Finish")
        track_line(robot2, 2)
        print("track_line Finish")
        time.sleep(0.5)
        catch_and_return(robot2)
        print("catch_and_return")
        track_line(robot2, 3)
        print("track_line Finish 276")
        execute_task_back(robot2, task)

        # print("F 265")
        # track_line(robot1, "1")
        print("The End")

    except KeyboardInterrupt:
        print("Emergency stop!")

    finally:
        # ep_vision.unsub_detect_info(name="line")
        # cv2.destroyAllWindows()
        # ep_camera.stop_video_stream()
        # robot1.close()
        robot2.close()
        print("All robots closed")

        robot1 = robot.Robot()
        robot1.initialize(conn_type="sta", sn="3JKDH6C001462K")
        ep1_vision, ep1_camera, ep1_chassis, ep1_gripper, ep1_sensor, ep1_gimbal = configure_robot(robot1)

        robot2 = robot.Robot()
        robot2.initialize(conn_type="sta", sn="3JKDH5D0017578")

        track_line(robot1, 1)
        ep1_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
        time.sleep(0.5)

        task = markers[0].info if markers else None
        execute_task(robot2, task)
        track_line(robot2, 2)
        time.sleep(0.5)
        catch_and_return(robot2)
        track_line(robot2, 3)
        execute_task_back(robot2, task)
        time.sleep(5)
        track_line(robot1, 1)
        print("The End")