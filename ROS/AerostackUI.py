from turtle import position
import rclpy
import threading
import time 

from websocket_client import WebSocketClient

import sys
sys.path.append('/home/rafa/aerostack2_ws/src/aerostack2/stack/user_interface/python_interface/python_interface')

from drone_interface import DroneInterface

class AerostackUI():
    def __init__(self):
        self.client = WebSocketClient("ws://127.0.0.1:8000/ws/user/")
        self.client.new_mission_function = self.new_mission_callback
        
        rclpy.init()
        
        self.drone_id = "drone_sim_14"
        self.drone_interface = DroneInterface(self.drone_id)
        
        time.sleep(1)
        
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        
    def new_mission_callback(self, msg):
        print("New mission received")
        print(msg)
        
    def run(self):
        odom = []
        while rclpy.ok():
            pose = self.drone_interface.get_gps_pose()
            
            orientation = self.drone_interface.get_orientation()
            
            odom.append([pose[0], pose[1]])
            
            print(f"Pose: {pose}")
            print(f"Orientation: {orientation}")
            print("Send msg")
            
            self.client.send_uav_info(
                {
                    'id': self.drone_id,
                    'state': 'fly', 
                    'pose': {'lat': pose[0], 'lng': pose[1], 'height': pose[2], 'yaw': orientation[2]},
                    'odom': odom 
                }
            )
            
            time.sleep(0.2)
        
if __name__ == '__main__':
    aerostackUI = AerostackUI()