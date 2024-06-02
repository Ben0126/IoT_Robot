import robomaster
pid_line = PIDCtrl()
list_LineList = RmList()
variable_x = 0
def start():
    global variable_x
    global list_LineList
    global pid_line
    robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
    gimbal_ctrl.rotate_with_degree(rm_define.gimbal_down,20)
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)
    pid_line.set_ctrl_params(330,0,28)
    while True:
        list_LineList=RmList(vision_ctrl.get_line_detection_info())
        if len(list_LineList) == 42:
            if list_LineList[2] >= 1:
                variable_x = list_LineList[19]
                pid_line.set_error(variable_x - 0.5)
                gimbal_ctrl.rotate_with_speed(pid_line.get_output(),0)
                chassis_ctrl.set_trans_speed(0.2)
                chassis_ctrl.move(0)
        else:
            gimbal_ctrl.rotate_with_speed(0,0)
