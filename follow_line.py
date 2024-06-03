import robomaster

# 初始化PID控制器
pid_line = PIDCtrl()

# 初始化用于存储线检测信息的列表
list_LineList = RmList()

# 定义一个变量用于存储x坐标误差
variable_x = 0


def start():
    global variable_x
    global list_LineList
    global pid_line

    # 设置机器人模式为底盘跟随模式
    robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
    # 云台向下旋转20度，以便摄像头能够看到地面上的线
    gimbal_ctrl.rotate_with_degree(rm_define.gimbal_down, 20)
    # 启用线检测功能
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    # 设置线的跟踪颜色为蓝色
    vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)

    # 设置PID控制器的参数
    pid_line.set_ctrl_params(330, 0, 28)

    while True:
        # 获取线检测的信息，并将其存储在list_LineList中
        list_LineList = RmList(vision_ctrl.get_line_detection_info())

        # 如果检测到的信息长度为42（表示检测到一条完整的线）
        if len(list_LineList) == 42:
            # 如果检测到的线段数量大于等于1
            if list_LineList[2] >= 1:
                # 获取线的x坐标
                variable_x = list_LineList[19]

                # 设置PID控制器的误差，目标位置为0.5（图像中心）
                pid_line.set_error(variable_x - 0.5)

                # 根据PID控制器的输出调整云台的旋转速度
                gimbal_ctrl.rotate_with_speed(pid_line.get_output(), 0)

                # 设置底盘的移动速度
                chassis_ctrl.set_trans_speed(0.2)

                # 使底盘前进
                chassis_ctrl.move(0)
        else:
            # 如果未检测到线，停止云台的旋转
            gimbal_ctrl.rotate_with_speed(0, 0)
