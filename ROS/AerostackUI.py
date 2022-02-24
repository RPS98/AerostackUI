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
        self.client.addMsgCallback('request', 'missionConfirm', self.new_mission_callback)
        self.client.addMsgCallback('request', 'missionStart', self.start_mission_callback)
        self.mission_list = []
        
        rclpy.init()
        
        self.drone_id = "drone_sim_14"
        self.drone_interface = DroneInterface(self.drone_id)
        
        time.sleep(1)
        
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        
    def start_mission_callback(self, msg, args):
        print("AerostackUI - Start mission")
        print(msg)
        
        
        
    def mission_planner(self, mission_id, uavList, layers):
        
        print("- Mission planner")
        print(uavList)
        print(layers)
        
        mission = []
        mission.append({'type': 'id', 'value': uavList[0]})
        
        for layer in layers:
            print(layer)
            type = layer['name']
            if (type == 'TakeOffPoint' or type == 'LandPoint'):
                mission.append(
                    {
                        'type': type, 
                        'value': [layer['values']['lat'], layer['values']['lng'], layer['height']]
                    }
                )
            elif (type == 'Path'):
                waypoints = []
                for point in layer['values']:
                    print("- Point")
                    print(point)
                    print([point['lat'], point['lng'], layer['height']])
                    waypoints.append([point['lat'], point['lng'], layer['height']])
                mission.append(
                    {
                        'type': type, 
                        'value': waypoints
                    }
                )
                
        if (mission[0]['type'] == 'id' and \
            mission[1]['type'] == 'TakeOffPoint' and \
            mission[2]['type'] == 'Path' and \
            mission[3]['type'] == 'LandPoint'):
            
            self.mission_list.append(
                {
                    'id': mission_id,
                    'mission': mission
                }
            )
            
            print("- Mission added")
            print(self.mission_list)
        
        else:
            print("- Invalid mission")
        
    def mission_interpreter(self, msg):
        
            print(f"- Mission interpreter")
            print(msg)
            
            confirm = 'confirmed'
            extra = []
            
            if msg['payload']['status'] == 'request':
                if len(msg['payload']['uavList']) == 0:
                    confirm = 'rejected'
                    extra.append('No UAVs')
                
                if len(msg['payload']['layers']) == 0:
                    confirm = 'rejected'
                    extra.append('No layers')
                    
                if msg['payload']['id'] != 'New Mission':
                    confirm = 'rejected'
                    extra.append('Invalid id, only "New Mission" is allowed for now :)')
            
            self.client.missionConfirm(
                self.client.mission_id,
                confirm,
                msg['payload']['id'],
                msg['from'],
                extra
            )
            
            self.client.mission_id += 1
            
            if confirm == 'confirmed':
                new_mission_info = msg['payload']
                new_mission_info['status'] = confirm
                new_mission_info['id'] = self.client.mission_id
                
                self.client.send_mission_info(new_mission_info)
                
                self.mission_planner(self.client.mission_id, msg['payload']['uavList'], msg['payload']['layers'])
        
        # self.mission_list.append(msg)
        
    def new_mission_callback(self, msg, args):
        print("AerostackUI - Confirm mission")
        print(self)
        print(msg)
        self.mission_interpreter(msg)
        
        
    def run(self):
        odom = []
        while rclpy.ok():
            #pose = self.drone_interface.get_gps_pose()
            #orientation = self.drone_interface.get_orientation()
            pose = [28.144099, -16.503337, 1, 0]
            orientation = [0, 0, 0, 0]
            
            odom.append([pose[0], pose[1]])
            
            # print(f"Pose: {pose}")
            # print(f"Orientation: {orientation}")
            # print("Send msg")
            
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