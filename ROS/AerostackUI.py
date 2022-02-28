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
        
        
        rclpy.init()
        
        self.drone_id = "drone_sim_14"
        self.drone_interface = DroneInterface(self.drone_id, False)
        self.mission_list = []
        self.speed = 2
        
        time.sleep(1)
        
        self.thread = threading.Thread(target=self.run)
        self.thread.start()    
        
        #time.sleep(1)
        #self.send_fake_mission()
           
    def send_fake_mission(self):
           
        self.fake_mission = {
            'status': 'confirmed', 
            'id': self.client.mission_id, 
            'uavList': [self.drone_id], 
            'layers': [
                {'name': 'TakeOffPoint', 'height': [3, 3], 'values':  {'lat': 28.144, 'lng': -16.503}}, 
                {'name': 'Path', 'height': [3, 3],         'values': [{'lat': 28.144, 'lng': -16.503}, {'lat': 28.144, 'lng': -16.5026}, {'lat': 28.1435, 'lng': -16.5026}, 
                                                                      {'lat': 28.144, 'lng': -16.503}]}, 
                {'name': 'LandPoint', 'height': [3, 3],    'values':  {'lat': 28.144, 'lng': -16.503}}]
        }
        
        self.client.send_mission_info(self.fake_mission)
        self.mission_planner(self.client.mission_id, self.fake_mission['uavList'], self.fake_mission['layers'])
        self.client.mission_id += 1
        
        
    def start_mission_callback(self, msg, args):
  
        for mission in self.mission_list:
            
            if (str(mission['id']) == str(msg['payload']['id'])):
                #print(f"- Start mission {mission['id']}")
                #print(mission['mission'])
                
                for element in mission['mission']:
                    if element['type'] == 'id':
                        # TODO: Select drone_interface instance for that UAV id
                        pass
                    else:
                        
                        if element['type'] == 'TakeOffPoint':
                            
                            waypoint =[
                                element['value'][0],
                                element['value'][1],
                                element['value'][2][1]
                            ]
                            
                            print(f"Send takeoff")
                            self.drone_interface.takeoff(height=waypoint[2])
                        
                        elif element['type'] == 'LandPoint':
                            waypoint =[
                                element['value'][0],
                                element['value'][1],
                                element['value'][2][1]
                            ]

                            print("Send land point: {waypoint}")
                            self.drone_interface.follow_gps_path([waypoint])

                            print("Send land")
                            self.drone_interface.land()
                            
                        elif element['type'] == 'Path':
                            waypoint = []
                            for point in element['value']:
                                waypoint.append([point[0], point[1], point[2][1]])
                            print(f"Send path: {waypoint}")
                            self.drone_interface.follow_gps_path(waypoint, self.speed)
                            
                        elif element['type'] == 'WayPoint':
                            waypoint =[
                                element['value'][0],
                                element['value'][1],
                                element['value'][2][1]
                            ]
                            print(f"Send waypoint: {waypoint}")
                            self.drone_interface.follow_gps_wp([waypoint], self.speed)
                            
                
        
    def mission_planner(self, mission_id, uavList, layers):
        """Convert Aerostack UI mission to ROS mission

        Args:
            mission_id (number): Mission id
            uavList (list): List of UAVs
            layers (list): List of layers
        """
        
        mission = []
        mission.append({'type': 'id', 'value': uavList[0]})
        
        for layer in layers:
            #print(layer)
            type = layer['name']
            if (type == 'TakeOffPoint' or type == 'LandPoint' or type == 'WayPoint'):
                mission.append(
                    {
                        'type': type, 
                        'value': [layer['values']['lat'], layer['values']['lng'], layer['height']]
                    }
                )
            elif (type == 'Path'):
                waypoints = []
                for point in layer['values']:
                    waypoints.append([point['lat'], point['lng'], layer['height']])
                mission.append(
                    {
                        'type': type, 
                        'value': waypoints
                    }
                )
                
        if (mission[0]['type'] == 'id' and \
            mission[1]['type'] == 'TakeOffPoint' and \
            mission[len(mission)-1]['type'] == 'LandPoint'):
            
            self.mission_list.append(
                {
                    'id': mission_id,
                    'mission': mission
                }
            )
        
        else:
            print("- Invalid mission")
        
    def mission_interpreter(self, msg):
        
            #print(f"- Mission interpreter")
            #print(msg)
            
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
        #print("AerostackUI - Confirm mission")
        #print(self)
        #print(msg)
        self.mission_interpreter(msg)
        
    def run(self):
        """ 
        pose = [28.144099, -16.503337, 1, 0]
        orientation = [0, 0, 0, 0]
        info = {
                'connected': True,
                'armed': True,
                'offboard': True,
                'state': 'hovering',
                'yaw_mode': 'rate',
                'control_mode': 'position',
                'reference_frame': 'world',
            }
        
        self.client.send_uav_info(
                {
                    'id': self.drone_id,
                    'state': info, 
                    'pose': {'lat': pose[0], 'lng': pose[1], 'height': pose[2], 'yaw': orientation[2]},
                    # 'odom': odom 
                }
            )
        """
        
        odom = []
        
        while rclpy.ok():
            
            pose = self.drone_interface.get_gps_pose()
            odom.append([pose[0], pose[1]])
            
            # self.plot_mission(path, odom)
            
            orientation = self.drone_interface.get_orientation()
            info = self.drone_interface.get_info()
            
            # print(f"Pose: {pose}")
            # print(f"Orientation: {orientation}")
            # print(f"Info: {info}")
            
            self.client.send_uav_info(
                {
                    'id': self.drone_id,
                    'state': info, 
                    'pose': {'lat': pose[0], 'lng': pose[1], 'height': pose[2], 'yaw': orientation[2]},
                    'odom': odom 
                }
            )
            
            time.sleep(0.1)
            
        
if __name__ == '__main__':
    aerostackUI = AerostackUI()