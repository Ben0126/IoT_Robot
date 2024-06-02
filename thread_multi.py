import threading
from robomaster import robot

def play_audio(robot_instance, filename):
    robot_instance.play_audio(filename=filename).wait_for_completed()

def initialize_and_play(robot_sn, audio_files):
    # 初始化机器人
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn=robot_sn)

    try:
        # 播放音频文件
        for audio_file in audio_files:
            play_audio(ep_robot, audio_file)
    finally:
        # 关闭机器人连接
        ep_robot.close()

if __name__ == '__main__':
    # 机器人序列号和音频文件
    robot1_sn = "3JKDH5D0017578"
    robot2_sn = "3JKDH6C001462K"
    audio_files = ["demo1.wav", "demo2.wav"]

    # 创建线程
    thread1 = threading.Thread(target=initialize_and_play, args=(robot1_sn, audio_files))
    thread2 = threading.Thread(target=initialize_and_play, args=(robot2_sn, audio_files))

    # 启动线程
    thread1.start()
    thread2.start()

    # 等待线程完成
    thread1.join()
    thread2.join()
