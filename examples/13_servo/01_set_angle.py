# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from robomaster import robot
import time


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKDH6C001462K")

    ep_servo = ep_robot.servo
    ep_gripper = ep_robot.gripper

    # # # 舵机1 -10 ~ 85 前後
    # ep_servo.moveto(index=0, angle=-10).wait_for_completed()
    # print("1")
    # time.sleep(2)
    # ep_servo.moveto(index=0, angle=80).wait_for_completed()
    # print("1")
    # time.sleep(2)
    # ep_servo.moveto(index=0, angle=30).wait_for_completed()
    # print("1")
    #
    #
    # # 舵机2 -95 ~ -35 上下
    # ep_servo.moveto(index=2, angle=-95).wait_for_completed()
    # print("2")
    # time.sleep(2)
    # ep_servo.moveto(index=2, angle=-35).wait_for_completed()
    # print("2")
    # time.sleep(2)
    # ep_servo.moveto(index=2, angle=-55).wait_for_completed()
    # print("2")

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

    ep_robot.close()


