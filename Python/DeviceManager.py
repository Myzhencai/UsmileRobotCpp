import websocket
import threading
# 管理与机械臂、相机、传感器的 WebSocket 通信与状态检测

class DeviceManager:
    def __init__(self):
        self.robot_ws = None
        self.camera_ws = None
        self.force_ws = None

    def connect_all(self):
        try:
            self.robot_ws = websocket.create_connection("ws://robot_ip:port")
            self.camera_ws = websocket.create_connection("ws://camera_ip:port")
            self.force_ws = websocket.create_connection("ws://force_sensor_ip:port")
            return True
        except Exception as e:
            print(f"[设备连接错误] {e}")
            return False

    def check_status(self):
        try:
            # 模拟检查心跳包或数据返回
            self.robot_ws.send("ping")
            return True
        except:
            return False

    def disconnect_all(self):
        if self.robot_ws: self.robot_ws.close()
        if self.camera_ws: self.camera_ws.close()
        if self.force_ws: self.force_ws.close()
