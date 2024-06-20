from http_controller import get_info, post_cmd, post_task, send_move_command
from mqtt_controller import MQTTController
import config
import keyboard
import json
import tkinter as tk
from tkinter import messagebox
import time
from red_detect_plus import detect_action
import threading
from shared import navigation_event, location
# 初始化速度和角速度
linear_speed = 0.5  # 假设的线性速度值
angular_speed = 0.5  # 假设的角速度值


# 定义按键响应函数


def on_key_event(e):
    if e.name == 'up':
        send_move_command(linear_x=linear_speed)
    elif e.name == 'down':
        send_move_command(linear_x=-linear_speed)
    elif e.name == 'left':
        send_move_command(angular_z=angular_speed)
    elif e.name == 'right':
        send_move_command(angular_z=-angular_speed)


# 选择获取那条信息
def get_user_info(options):
    print("请选择一个选项：")
    for i, option in enumerate(options, 1):
        print(f"{i}. {option}")
    choice = input("请输入选项编号: ")
    try:
        # 将用户输入的编号转换为整数，并从字典中获取相应的get_data值
        selected_option = list(options.keys())[int(choice) - 1]
        return options[selected_option]
    except (IndexError, ValueError):
        print("无效的输入，请输入正确的选项编号。")
        return None


# get部分处理#******************************************************#get部分处理#
# 处理 信息1返回数据
def handle_map_info(data):
    map_infos = []
    for map_info in data:
        map_uuid = map_info.get("map_uuid")
        map_name = map_info.get("map_name")
        map_infos.append((map_uuid, map_name))
    return map_infos


# 处理 信息2 返回数据
def handle_location_info(data):
    locations_info = []
    for map_info in data:
        map_uuid = map_info.get("map_uuid")
        locations = map_info.get("loc_list", [])
        for loc in locations:
            location_uuid = loc.get("location_uuid")
            location_name = loc.get("location_name")
            locations_info.append((map_uuid, location_uuid, location_name))
    return locations_info


# 处理 信息3 返回数据
def handle_current_info(data):
    map_infos = []
    map_uuid = data.get("map_uuid")
    map_name = data.get("map_name")
    map_infos.append((map_uuid, map_name))
    return map_infos


# task部分处理#******************************************************#task部分处理#
def post_send_command(command_name, **kwargs):
    # 构造命令
    command = {
        "command_name": command_name,
        **kwargs  # 添加额外的命令参数
    }
    # 将命令转换为JSON字符串
    command_json = json.dumps(command)
    # 设置请求的form-data
    form_data = {"json": command_json}
    return form_data


# 处理接收返回数据
def handle_task_info(data):
    error_code = data.get("error_code")
    if error_code == 0:
        print("\n任务执行成功")
    else:
        print("\n任务执行失败")


def wait_with_key_check(timeout, key):
    key_pressed = threading.Event()

    def check_key():
        while not key_pressed.is_set():
            if input() == key:
                key_pressed.set()
                print(f"{key}键被按下, 提前退出等待。")
    key_check_thread = threading.Thread(target=check_key)
    key_check_thread.daemon = True
    key_check_thread.start()
    start_time = time.time()
    while time.time() - start_time < timeout:
        if key_pressed.is_set():   
            return True
        time.sleep(0.05)  # 检查按键的时间间隔，可根据需要调整
    return False

options_get = {
    "获取全部地图信息": "map_get_all_info_list",
    "获取全部地图的定位点": "location_get_list_of_all_map",
    "获取当前地图": "get_current_map",
}

options_task = {
    "导航去当前地图指定定位点": "navigation_start",
    "停止导航任务": "navigation_pause",
    "恢复导航任务": "navigation_resume",
    "取消导航任务": "navigation_cancel",
    "设置导航任务使用的地图": "navigation_set_map",
    "开始对接充电桩": "dock_start",
    "取消对接充电桩": "dock_cancel",
    "开始脱离充电桩": "undock_start",
}


def main():
    # 初始化MQTT
    mqtt_controller = MQTTController(config.MQTT_BROKER, config.MQTT_PORT)
    mqtt_controller.connect()
    navigation_points = {
        # "16c1401619144b35952f2b7d144e05d4": "窗户", 
        "d48806f38b0946fc9fd381a61791936e": "门口","c74d446ae58a45969f5758f892800964":"餐桌"}  # 假设这是两个导航点的 UUID[0起点  1窗户]
    current_uuid_index = 0  # 用于跟踪当前导航点的索引
    navigation_location_uuids = list(navigation_points.keys())  # 将字典键转换为列表
    exit_loop = False  # 用于控制循环退出的标志
    # navigation_location_uuid = "d60bef6ca8c542b3aeee0d0a8c308ae3"
    # 初始化变量

    while True:
        print("\n主菜单:\n")
        print("1. 获取信息")
        print("2. 发送任务")
        print("3. 手动控制移动")
        print("4. 退出")
        choice = input("请输入选项 (1/2/3/4): ")
        if choice == "1":
            while True:
                print("\n输入其他编号则返回")
                get_choice = get_user_info(options_get)
                if get_choice:
                    response = get_info(get_choice)
                    if get_choice == "map_get_all_info_list":
                        map_infos = handle_map_info(response["data"])
                        for map_uuid, map_name in map_infos:
                            print(f"Map_uuid: {map_uuid}, Name: {map_name}")
                    elif get_choice == "location_get_list_of_all_map":
                        location_infos = handle_location_info(response["data"])
                        for map_uuid, location_uuid, location_name in location_infos:
                            print(
                                f"Map_uuid: {map_uuid}, Location_uuid: {location_uuid}, Name: {location_name}")
                    elif get_choice == "get_current_map":
                        map_infos = handle_current_info(response["data"])
                        for map_uuid, map_name in map_infos:
                            print(f"Map_uuid: {map_uuid}, Name: {map_name}")
                else:
                    break
        elif choice == "2":
            while True:
                print("\n输入其他编号则返回")
                task_choice = get_user_info(options_task)
                print(task_choice)
                if task_choice == "navigation_set_map":
                    task_data = post_send_command(
                        task_choice, set_map_uuid="70a15161e99a42dd809006f298658a6d", set_location_uuid="b3060551550c45a09c3a8353d8fdcaf4")
                    response = post_task(task_data)
                    handle_task_info(response)
                elif task_choice == "dock_start":
                    task_data = post_send_command(task_choice)
                    response = post_task(task_data)
                    handle_task_info(response)
                elif task_choice == "undock_start":
                    task_data = post_send_command(task_choice)
                    response = post_task(task_data)
                    handle_task_info(response)
                elif task_choice == "dock_cancel":
                    task_data = post_send_command(task_choice)
                    response = post_task(task_data)
                    handle_task_info(response)
                elif task_choice == "navigation_start":
                    exit_loop = False
                    while not exit_loop:
                        target_location = navigation_points[navigation_location_uuids[current_uuid_index]]
                        print(
                            f"即将导航到 {navigation_points[navigation_location_uuids[current_uuid_index]]}...")
                        navigation_location_uuid = navigation_location_uuids[current_uuid_index]
                        current_uuid_index = (
                            current_uuid_index + 1) % len(navigation_location_uuids)
                        
                        # 发送导航任务
                        task_data = post_send_command(
                            task_choice, location_uuid=navigation_location_uuid)
                        response = post_task(task_data)

                        navigation_event.clear()
                        MQTTController.back_location(target_location)
                        # 等待一段时间或直到到达目的地
                        navigation_event.wait()
                        # while navigation_status_flag:
                        #     print(f"正在导航中...")
                        #     time.sleep(1)
                        exit_loop = wait_with_key_check(10, 'e')
                        if exit_loop:  # 检查 esc 键是否被按下
                            print("收到退出指令，停止导航任务。")
                            break
                        handle_task_info(response)
                        print(
                            f"到达 {navigation_points[navigation_location_uuid]}，等待 30 秒...")
                    choice = 2
                elif task_choice == "navigation_pause":
                    task_data = post_send_command(task_choice)
                    response = post_task(task_data)
                    handle_task_info(response)
                elif task_choice == "navigation_cancel":
                    task_data = post_send_command(task_choice)
                    response = post_task(task_data)
                    handle_task_info(response)
                elif task_choice == "navigation_resume":
                    task_data = post_send_command(task_choice)
                    response = post_task(task_data)
                    handle_task_info(response)

                else:
                    break
        elif choice == "3":
           # 监听箭头键
            keyboard.on_press_key("up", on_key_event)
            keyboard.on_press_key("down", on_key_event)
            keyboard.on_press_key("left", on_key_event)
            keyboard.on_press_key("right", on_key_event)
            keyboard.wait('esc')
        elif choice == "4":
            break
        else:
            print("无效选项，请重新选择。")
    mqtt_controller.disconnect()  # 断开MQTT连接


if __name__ == "__main__":
    main()
