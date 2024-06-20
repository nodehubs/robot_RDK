import paho.mqtt.client as mqtt
import json
import config
from red_detect_plus import detect_action
from shared import navigation_event, location
import time
last_estop_status = None
last_battery_status = None
# last_navigation_status = None
navigation_status_flag = None


class MQTTController:

    def __init__(self, broker, port):
        self.broker = broker
        self.port = port
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        # 订阅你需要的topic
        client.subscribe(config.MQTT_TOPIC)

    def back_location(target_location):
        global location
        location = target_location

    def on_message(self, client, userdata, msg):
        # 将消息负载解码为字符串
        global last_estop_status
        global last_battery_status
        global last_navigation_status
        global navigation_status_flag
        global location
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            payload_data = payload.get("data", {})
            # 尝试将字符串解析为JSON
            if payload.get("command_name") == "estop_status":
                estop_status = payload_data.get("is_triggered")
                if estop_status != last_estop_status:
                    print(f"\n急停状态: {estop_status}")
                    last_estop_status = estop_status
            elif payload.get("command_name") == "battery_data":
                soc_percent = payload_data.get("soc_percent")
                if soc_percent != last_battery_status:
                    print(f"\n电池电量:{soc_percent}")
                    last_battery_status = soc_percent
            elif payload.get("command_name") == "navigation_status":
                navigation_status = payload_data.get("activated")
                navigation_status_flag = not navigation_status
                print(f"\n导航状态标志:{navigation_status_flag}")
                print(f"\n导航目标位置:{location}")
                if navigation_status_flag and location == "餐桌":
                    time.sleep(4)
                    # detect_action()
                    navigation_event.set()
                elif navigation_status_flag:
                    navigation_event.set()

        except json.JSONDecodeError:
            print("无法解析消息为JSON格式")
        # print(f"Topic: {msg.topic}\nMessage: {msg.payload}")

    def connect(self):
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()
        print("MQTT 连接")

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
