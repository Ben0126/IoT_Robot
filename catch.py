from robomaster import robot
import time

# 初始化一個變數來保存距離數據
distance_data = None

# 回調函數來處理距離數據
def sub_data_handler(sub_info):
    global distance_data
    distance = sub_info
    distance_data = distance[0]
    print("tof1:{0}".format(distance_data))

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")

    ep_servo = ep_robot.servo
    ep_gripper = ep_robot.gripper
    ep_sensor = ep_robot.sensor 
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal

    ep_sensor.sub_distance(freq=5, callback=sub_data_handler) 

    ep_servo.moveto(index=1, angle=-10).wait_for_completed()
    ep_servo.moveto(index=2, angle=-35).wait_for_completed()
    time.sleep(1)
    ep_gripper.open(power=10)
    time.sleep(1)

    while True:
        if distance_data is not None and distance_data <= 50:
            ep_chassis.drive_speed(x=0, y=0, z=0)
            time.sleep(1)
            ep_chassis.move(x=0.05, y=0, z=0, xy_speed=0.01).wait_for_completed()
            break
        ep_chassis.drive_speed(x=0.1, y=0, z=0)
        time.sleep(0.5) 

    print("2")
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
    time.sleep(1)
    print("Finish")
    ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed()
    ep_gimbal.moveto(pitch=0, yaw=0,pitch_speed=100, yaw_speed=100).wait_for_completed()
    ep_robot.close()
