# coding=utf8
import time
import numpy as np
import socket
import json


class Robot:
    def __init__(self, tcp_host_ip="192.168.199.100", tcp_port=8055):
        self.ip = tcp_host_ip
        self.port = tcp_port
        self.sock = None
        self.connect()
        self.enable_servo()

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.ip, self.port))
            print("Connected to robot")
        except Exception as e:
            print("连接失败:", e)
            self.sock.close()
            self.sock = None

    def disconnect(self):
        if self.sock:
            self.sock.close()
            print("Disconnected from robot")

    def send_command(self, cmd, params=None, id=1, ret_flag=True):
        if not params:
            params = []
        else:
            params = json.dumps(params)
        send_str = f'{{"method":"{cmd}","params":{params},"jsonrpc":"2.0","id":{id}}}\n'
        self.sock.sendall(bytes(send_str, "utf-8"))
        if ret_flag:
            ret = self.sock.recv(1024)
            return json.loads(str(ret, "utf-8"))

    def enable_servo(self):
        self.send_command("set_servo_status", {"status": 1})
        time.sleep(1)  # 等待命令生效

    def get_current_pose(self):
        response = self.send_command(
            "get_tcp_pose", {"coordinate_num": -1, "tool_num": 0, "unit_type": 0})
        if response and 'result' in response:
            print("Current Pose:", response['result'])
            return response['result']  # 返回位姿数据而不是打印
        else:
            print("Failed to get current pose")
            return None  # 如果未成功获取位姿，则返回 None

    def move_j(self, joint):
        joint_deg = np.rad2deg(joint)
        self.send_command("moveByJoint", {
                          "targetPos": joint_deg.tolist(), "speed": 30, "acc": 10, "dec": 10})

    def open_gripper(self):
        self.send_command("setSysVarB", {"addr": 1, "value": 100})
        self.send_command("setSysVarB", {"addr": 0, "value": 1})
        print("夹爪已打开！")

    def close_gripper(self):
        self.send_command("setSysVarB", {"addr": 1, "value": 5})
        self.send_command("setSysVarB", {"addr": 0, "value": 1})
        print("夹爪已闭合！")


if __name__ == "__main__":
    robot_ip = "192.168.199.100"
    robot = Robot(robot_ip)

    try:
        robot.get_current_pose()
        # robot.move_j([0, -np.pi / 2, np.pi / 2, -
        #              np.pi / 2, np.pi / 2, -np.pi / 2])
        # robot.open_gripper()
        # robot.close_gripper()
    finally:
        robot.disconnect()
