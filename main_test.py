import cv2
from robomaster import robot, vision
import numpy as np
import time
import logging

logging.basicConfig(level=logging.INFO)

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

markers = []
line_list = []
distance_data = None

def on_detect_marker(marker_info):
    global markers
    markers = [MarkerInfo(x, y, w, h, info) for x, y, w, h, info in marker_info]
    logging.info("Detected markers: %s", [m.info for m in markers])

def on_detect_line(line_info):
    global line_list
    line_list = [PointInfo(x, y, ceta, c) for x, y, ceta, c in line_info[1:]]

def sub_data_distance(sub_info):
    global distance_data
    distance_data = sub_info[0]
    logging.info("TOF distance: %s", distance_data)

def configure_robot(ep_robot):
    return (ep_robot.vision, ep_robot.camera, ep_robot.chassis, ep_robot.gripper, ep_robot.sensor, ep_robot.gimbal)

def track_line(ep_robot, mode):
    ep_vision, ep_camera, ep_chassis, ep_gripper, ep_sensor, ep_gimbal = configure_robot(ep_robot)
    ep_camera.start_video_stream(display=False)

    try:
        if mode in [1, 3]:
            configure_and_execute_mode(ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, mode, "marker")
        elif mode == 2:
            configure_and_execute_mode(ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, mode, "line")

    except Exception as e:
        logging.error("An error occurred: %s", e)
    finally:
        ep_vision.unsub_detect_info(name="line")
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        ep_robot.close()

def configure_and_execute_mode(ep_vision, ep_chassis, ep_camera, ep_gimbal, ep_gripper, ep_sensor, mode, detect_type):
    if detect_type == "marker":
        ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
    else:
        ep_sensor.sub_distance(freq=5, callback=sub_data_distance)
        ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
        ep_gimbal.moveto(pitch=-50, yaw=0).wait_for_completed()

    pid = PIDController(Kp=33, Ki=0, Kd=0.5)
    x_val = 0.5 if mode != 2 else 0.1

    while True:
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        frame_width = img.shape[1]

        if mode == 1 or mode == 3:
            process_markers(ep_chassis, img)
            process_lines(ep_chassis, ep_gimbal, img, pid, x_val, frame_width)
        elif mode == 2:
            process_lines(ep_chassis, ep_gimbal, img, pid, x_val, frame_width)
            if distance_data is not None and distance_data <= 50:
                stop_and_reposition(ep_chassis)
                break

        cv2.imshow("Line", img)
        if cv2.waitKey(1) & 0xFF == ord(' '):
            stop_and_reposition(ep_chassis)
            break

def process_markers(ep_chassis, img):
    for marker in markers:
        cv2.rectangle(img, marker.pt1, marker.pt2, (255, 255, 255))
        cv2.putText(img, marker.info, marker.center, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

    if markers:
        stop_and_reposition(ep_chassis)
        return True
    return False

def process_lines(ep_chassis, ep_gimbal, img, pid, x_val, frame_width):
    if line_list:
        recent_points = line_list[-10:]
        avg_x = int(np.mean([p.pt[0] for p in recent_points]))
        avg_y = int(np.mean([p.pt[1] for p in recent_points]))

        setpoint = frame_width // 2
        control_signal = pid.compute(setpoint, avg_x) / 100

        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=control_signal)
        ep_chassis.drive_speed(x=x_val, y=0, z=control_signal)

        for point in line_list:
            cv2.circle(img, point.pt, 3, point.color, -1)
        cv2.circle(img, (avg_x, avg_y), 5, (0, 255, 0), -1)

def stop_and_reposition(ep_chassis):
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.5)
    time.sleep(1)

def main():
    try:
        robot1 = robot.Robot()
        robot1.initialize(conn_type="sta", sn="3JKDH6C001462K")
        robot2 = robot.Robot()
        robot2.initialize(conn_type="sta", sn="3JKDH5D0017578")

        track_line(robot1, 1)
        ep1_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()

        task = markers[0].info if markers else None
        execute_task(robot2, task)

        track_line(robot2, 2)
        catch_and_return(robot2)

        track_line(robot1, 1)
        logging.info("The End")

    except KeyboardInterrupt:
        logging.warning("Emergency stop!")

    finally:
        robot1.close()
        robot2.close()
        logging.info("All robots closed")

def execute_task(robot, task):
    ep_chassis, ep_servo, ep_gimbal = robot.chassis, robot.servo, robot.gimbal
    tasks = {
        "1": lambda: ep_chassis.move(x=0, y=0, z=90, z_speed=45).wait_for_completed(),
        "2": lambda: ep_chassis.move(x=0, y=0, z=0, xy_speed=0).wait_for_completed(),
        "3": lambda: ep_chassis.move(x=0, y=0, z=-90, z_speed=45).wait_for_completed(),
    }
    tasks.get(task, lambda: stop_and_reposition(ep_chassis))()
    ep_servo.moveto(index=1, angle=-10).wait_for_completed()
    ep_servo.moveto(index=2, angle=-35).wait_for_completed()
    ep_gimbal.moveto(pitch=-50, yaw=0).wait_for_completed()

def catch_and_return(robot):
    ep_chassis, ep_gripper, ep_servo, ep_gimbal = robot.chassis, robot.gripper, robot.servo, robot.gimbal
    ep_gripper.close(power=10)
    time.sleep(1.5)
    ep_gripper.pause()
    ep_servo.moveto(index=2, angle=-95).wait_for_completed()
    ep_servo.moveto(index=1, angle=80).wait_for_completed()
    time.sleep(2)
    ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
    ep_gimbal.moveto(pitch=-50, yaw=0, pitch_speed=100, yaw_speed=100).wait_for_completed()

if __name__ == "__main__":
    main()
