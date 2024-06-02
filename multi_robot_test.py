from robomaster import robot
from robomaster import camera
import time

def sub_info_handler(batter_info, ep_robot):
    percent = batter_info
    print("Battery: {0}%.".format(percent))
    ep_led = ep_robot.led
    brightness = int(percent * 255 / 100)
    ep_led.set_led(comp="all", r=brightness, g=brightness, b=brightness)

def battery_info(robot):
    ep_battery = robot.battery
    ep_battery.sub_battery_info(5, sub_info_handler, robot)
    time.sleep(1)
    ep_battery.unsub_battery_info()

def camera_display(robot):
    ep_camera = robot.camera
    # 显示十秒图传
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)
    time.sleep(5)
    ep_camera.stop_video_stream()

def arm(robot):
    ep_servo = robot.servo
    ep_gripper = robot.gripper
    ep_servo.moveto(index=1, angle=-10).wait_for_completed()
    ep_servo.moveto(index=2, angle=-35).wait_for_completed()

    ep_gripper.open(power=10)
    time.sleep(2)
    ep_gripper.pause()
    time.sleep(1)
    print("open")

    ep_gripper.close(power=10)
    time.sleep(1.5)
    ep_gripper.pause()
    time.sleep(1)
    print("close")
    ep_servo.moveto(index=2, angle=-95).wait_for_completed()
    ep_servo.moveto(index=1, angle=80).wait_for_completed()
    time.sleep(2)
    print("Finish")

if __name__ == '__main__':
    # 初始化机器人
    robot1 = robot.Robot()
    robot2 = robot.Robot()

    # 3JKDH5D0017578 : Arm; 3JKDH6C001462K : Cargo
    robot1.initialize(conn_type="sta", sn="3JKDH5D0017578")
    robot2.initialize(conn_type="sta", sn="3JKDH6C001462K")

    arm(robot1)
    time.sleep(2)
    battery_info(robot1)

    print("------")

    arm(robot2)
    time.sleep(2)
    battery_info(robot2)

    robot1.close()
    robot2.close()

