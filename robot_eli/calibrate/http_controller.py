import requests
import json
import config
headers = config.HEADERS


def get_info(get_data):
    url = f"{config.BASE_URL}/{get_data}"
    response = requests.get(url, headers=headers)
    if response.status_code == 200:
        # 解析响应内容
        return response.json()
    else:
        response.raise_for_status()


def post_cmd(cmd_data):
    url = f"{config.BASE_URL}/cmd"
    response = requests.post(url, data=cmd_data, headers=headers)
    if response.status_code == 200:
        return response.json()
    else:
        response.raise_for_status()


def send_move_command(linear_x=0.0, angular_z=0.0):
    """发送移动指令到机器人"""
    command = {
        "command_name": "move_twist",
        "linear_x": linear_x,
        "linear_y": 0.0,
        "linear_z": 0.0,
        "angular_x": 0.0,
        "angular_y": 0.0,
        "angular_z": angular_z
    }
    command_json = json.dumps(command)
    # 设置请求的form-da
    form_data = {"json": command_json}
    url = f"{config.BASE_URL}/cmd"
    response = requests.post(url, headers=headers, data=form_data)
    print("命令发送:", command, "响应:", response.text)


def post_task(task_data):
    url = f"{config.BASE_URL}/task"
    response = requests.post(url, headers=headers, data=task_data)
    if response.status_code == 200:
        print("请求成功，响应内容：", response.json())
        return response.json()
    else:
        print("请求失败，响应内容：", response.raise_for_status())
        response.raise_for_status()
