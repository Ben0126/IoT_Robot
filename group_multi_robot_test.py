from multi_robomaster import multi_robot
import time

def group_task_turn(robot_group):
    z = 90

    # 右转 90度
    robot_group.chassis.move(0, 0, -z, 2, 180).wait_for_completed()
    # 左转 90度
    robot_group.chassis.move(0, 0, z, 2, 180).wait_for_completed()

def group_task_gripper(robot_group):
    ep_gripper = robot_group.gripper
    ep_gripper.open()
    time.sleep(3)
    ep_gripper.close()
    time.sleep(3)

def group_arm(robot_group):
    ep_servo = robot_group.servo
    ep_gripper = robot_group.gripper
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
    # 3JKDH5D0017578 : Arm; 3JKDH6C001462K : Cargo

    robots_sn_list = ['3JKDH5D0017578', '3JKDH6C001462K']
    multi_robots = multi_robot.MultiEP()
    multi_robots.initialize()
    number = multi_robots.number_id_by_sn([0, robots_sn_list[0]], [1, robots_sn_list[1]])
    
    print("The number of robot is: {0}".format(number))
    
    robot_group_arm = multi_robots.build_group([0])
    robot_group_cargo = multi_robots.build_group([1])
    robot_group_all = multi_robots.build_group([0, 1])

    multi_robots.run([robot_group_all, group_task_gripper])
    # multi_robots.run([robot_group_cargo, group_task])
    print("Game over")
    multi_robots.close()


