import time
import threading
from pyrsistent import inc
import websocket
from websocket_client import WebSocketClient


class AerostackUI():
    def __init__(self):
        # self.client = WebSocketClient("ws://192.168.30.23:8000/ws/user/")
        self.client = WebSocketClient("ws://127.0.0.1:8000/ws/user/")
        
        self.client.addMsgCallback('request', 'missionConfirm', self.new_mission_callback)
        self.client.addMsgCallback('request', 'missionStart', self.start_mission_callback)

        self.drone_id = "drone_sim_rafa_0" # "drone_sim_14"
        # self.uav_id_list = ['drone_sim_rafa_0']
        self.uav_id_list = ['drone_sim_rafa_0', 'drone_sim_rafa_1', 'drone_sim_rafa_2']
        # self.uav_id_list = ['drone_sim_rafa_0', 'drone_sim_rafa_1']
        self.mission_list = []
        self.speed = 2
        
        time.sleep(1)
        
        self.thread = threading.Thread(target=self.run)
        self.thread.start()    
        
        time.sleep(2)
        msg = {'type': 'request', 'header': 'missionConfirm', 'status': 'request', 'payload': {'status': 'request', 'id': 'New Mission', 'uavList': ['drone_sim_rafa_0', 'drone_sim_rafa_1'], 'layers': [{'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.14386, 'lng': -16.50225}}, {'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_1'], 'values': {'lat': 28.14396, 'lng': -16.50215}}, {'name': 'Area', 'height': [3, 3], 'uavList': ['auto'], 'values': [[{'lat': 28.144016372555452, 'lng': -16.503212302923206}, {'lat': 28.143671070700133, 'lng': -16.50311574339867}, {'lat': 28.143680531039788, 'lng': -16.50249615311623}, {'lat': 28.144023467787367, 'lng': -16.5025806427002}]]}, {'name': 'LandPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.143912309099793, 'lng': -16.50326326489449}}, {'name': 'LandPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_1'], 'values': {'lat': 28.143997451934666, 'lng': -16.50326862931252}}]}, 'from': 2, 'to': None}
        self.mission_interpreter(msg)
        
    def send_fake_confirm_mission(self):
        self.client.missionConfirm(
            self.client.mission_id,
            'confirmed',
            'New mission',
            '2',
            []
        )
          
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
        
        send_mission = {'status': 'confirmed', 'id': 1, 'uavList': ['drone_sim_rafa_0'], 'layers': [{'name': 'TakeOffPoint', 'height': [5, 5], 'values': {'lat': 28.1439938, 'lng': -16.5032927}}, {'name': 'Path', 'height': [5, 5], 'values': [{'lat': 28.143967888457993, 'lng': -16.503202915191654}, {'lat': 28.143970253536445, 'lng': -16.502602100372318}, {'lat': 28.143880380519278, 'lng': -16.502556502819065}, {'lat': 28.14387919797907, 'lng': -16.503158658742908}]}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.143779864555484, 'lng': -16.50313183665276}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.14379287250906, 'lng': -16.50254845619202}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.143711277137808, 'lng': -16.502514928579334}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.143694721547664, 'lng': -16.503106355667118}}, {'name': 'LandPoint', 'height': [5, 5], 'values': {'lat': 28.14369708663212, 'lng': -16.503161340951923}}]}

        
        self.client.send_mission_info(send_mission)
        self.mission_planner(1, [self.drone_id], 
                            [{'name': 'TakeOffPoint', 'height': [5, 5], 'values': {'lat': 28.1439938, 'lng': -16.5032927}}, {'name': 'Path', 'height': [5, 5], 'values': [{'lat': 28.143967888457993, 'lng': -16.503202915191654}, {'lat': 28.143970253536445, 'lng': -16.502602100372318}, {'lat': 28.143880380519278, 'lng': -16.502556502819065}, {'lat': 28.14387919797907, 'lng': -16.503158658742908}]}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.143779864555484, 'lng': -16.50313183665276}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.14379287250906, 'lng': -16.50254845619202}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.143711277137808, 'lng': -16.502514928579334}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.143694721547664, 'lng': -16.503106355667118}}, {'name': 'LandPoint', 'height': [5, 5], 'values': {'lat': 28.14369708663212, 'lng': -16.503161340951923}}]
                             )
        self.client.mission_id += 1
        
        
    def start_mission_callback(self, msg, args):
  
        for mission in self.mission_list:
            
            if (str(mission['id']) == str(msg['payload']['id'])):
                print("- Start mission ", mission['id'])
                print(mission['mission'])
                
                for element in mission['mission']:
                    if element['type'] == 'id':
                        # TODO: Select drone_interface instance for that UAV id
                        pass
                    else:
                        pass
                        # if element['type'] == 'TakeOffPoint':
                            
                        #     waypoint =[
                        #         element['value'][0],
                        #         element['value'][1],
                        #         element['value'][2][1]
                        #     ]
                            
                        #     print(f"Send takeoff")
                        #     self.drone_interface.takeoff(height=waypoint[2])
                        
                        # elif element['type'] == 'LandPoint':
                        #     waypoint =[
                        #         element['value'][0],
                        #         element['value'][1],
                        #         element['value'][2][1]
                        #     ]

                        #     print("Send land point: {waypoint}")
                        #     self.drone_interface.follow_gps_path([waypoint])

                        #     print("Send land")
                        #     self.drone_interface.land()
                            
                        # elif element['type'] == 'Path':
                        #     waypoint = []
                        #     for point in element['value']:
                        #         waypoint.append([point[0], point[1], point[2][1]])
                        #     print(f"Send path: {waypoint}")
                        #     self.drone_interface.follow_gps_path(waypoint, self.speed)
                            
                        # elif element['type'] == 'WayPoint':
                        #     waypoint =[
                        #         element['value'][0],
                        #         element['value'][1],
                        #         element['value'][2][1]
                        #     ]
                        #     print(f"Send waypoint: {waypoint}")
                        #     self.drone_interface.follow_gps_wp([waypoint], self.speed)
    
    def swarm(self, uavListAux, mission, sublist, first_uav):
        print("Swarm")
        print(uavListAux)
        print(mission)
        print(sublist)
        initial_pose = {}
        for uav in uavListAux:
            initial_pose[uav] = mission[uav][len(mission[uav])-1]['values'][len(mission[uav][len(mission[uav])-1]['values'])-1]
        
        for layer in sublist:
            uavList = layer['uavList']
            if (type == 'TakeOffPoint' or type == 'LandPoint' or type == 'WayPoint'):
                uav = uavList[0]
                if uav == 'auto':
                    uav = first_uav
                
            elif (type == 'Path'):
                uav = uavList[0]
                if uav == 'auto':
                    uav = first_uav
                
                waypoints = []
                for point in layer['values']:
                    waypoints.append([point['lat'], point['lng'], layer['height'][1]])
                
                mission[uav].append({
                    'type': type,
                    'values': waypoints
                })
                
                layer['uavList'] = [uav],
            
            elif type == 'Area':
                pass
                #TODO: Manage two areas 
            
            else:
                raise Exception("Unknown layer type")
            
        print("initial_pose: ")
        print(initial_pose)
        print("End initial pose")
            
        return mission
                
        
    def mission_planner(self, mission_id, confirm, mission_info):
        """Convert Aerostack UI mission to ROS mission

        Args:
            mission_id (number): Mission id
            uavList (list): List of UAVs
            layers (list): List of layers
        """
        
        print("AerostackUI - Mission planner")
        print(mission_id)
        print(confirm)
        print(mission_info)
        
        send_mission = {
            'id': mission_id,
            'status': confirm,
            'uavList': mission_info['uavList'],
            'layers': []
        }
        
        first_uav = mission_info['uavList'][0]
        
        mission = {}
        for uav in mission_info['uavList']:
            mission[str(uav)] = []
    
        print("For each layer")
        for layer in mission_info['layers']:
            print(layer)
            type = layer['name']
            uavList = layer['uavList']
            height = layer['height']
            values = layer['values']
            
            layer = {
                'name': type,
                'uavList': uavList,
                'height': height,
                'values': values
            }
            
            if (type == 'TakeOffPoint' or type == 'LandPoint' or type == 'WayPoint'):
                uav = uavList[0]
                if uav == 'auto':
                    uav = first_uav
                    
                mission[uav].append({
                    'type': type,
                    'values': [[values['lat'], values['lng'], height[1]]]
                })
                layer['uavList'] = [uav]
                
            elif (type == 'Path'):
                uav = uavList[0]
                if uav == 'auto':
                    uav = first_uav
                
                waypoints = []
                for point in layer['values']:
                    waypoints.append([point['lat'], point['lng'], layer['height'][1]])
                
                mission[uav].append({
                    'type': type,
                    'values': waypoints
                })
                
                layer['uavList'] = [uav],
            
            elif type == 'Area':
                
                if uavList[0] == 'auto':
                    uavListAux = mission_info['uavList']
                else:
                    uavListAux = uavList
                
                index = mission_info['layers'].index(layer)
                range_to_end = range(index, len(mission_info['layers']))
                sublist = [mission_info['layers'][i] for i in range_to_end]
                
                mission = self.swarm(uavListAux, mission, sublist, first_uav)
                
                layer['uavList'] = uavListAux
            
            else:
                raise Exception("Unknown layer type")
            
            send_mission['layers'].append(layer)
            
            # if (type == 'TakeOffPoint' or type == 'LandPoint' or type == 'WayPoint'):
            #     mission.append(
            #         {
            #             'type': type, 
            #             'value': [layer['values']['lat'], layer['values']['lng'], layer['height']]
            #         }
            #     )
            # elif (type == 'Path'):
            #     waypoints = []
            #     for point in layer['values']:
            #         waypoints.append([point['lat'], point['lng'], layer['height']])
            #     mission.append(
            #         {
            #             'type': type, 
            #             'value': waypoints
            #         }
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
            
        """
        
    def mission_interpreter(self, msg):
        
        print(f"- Mission interpreter")
        print(msg)
        
        """
        {'type': 'request', 'header': 'missionConfirm', 'status': 'request', 'payload': {'status': 'request', 'id': 'New Mission', 'uavList': ['drone_sim_rafa_1', 'drone_sim_rafa_0'], 'layers': [{'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_1'], 'values': {'lat': 28.14396, 'lng': -16.50215}}, {'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.14386, 'lng': -16.50225}}, {'name': 'Area', 'height': [3, 3], 'uavList': ['auto'], 'values': [[{'lat': 28.144045936018713, 'lng': -16.502568572759632}, {'lat': 28.14376685660033, 'lng': -16.502509564161304}, {'lat': 28.143753848643602, 'lng': -16.503182798624042}, {'lat': 28.144006912245466, 'lng': -16.503237783908848}]]}, {'name': 'LandPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_1'], 'values': {'lat': 28.143783412179328, 'lng': -16.503265947103504}}, {'name': 'LandPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.143912309099793, 'lng': -16.503300815820698}}]}, 'from': 2, 'to': None}
        
        {'type': 'request', 'header': 'missionConfirm', 'status': 'request', 'payload': {'status': 'request', 'id': 'New Mission', 'uavList': ['drone_sim_rafa_0', 'drone_sim_rafa_1'], 'layers': [{'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.14386, 'lng': -16.50225}}, {'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_1'], 'values': {'lat': 28.14396, 'lng': -16.50215}}, {'name': 'Area', 'height': [3, 3], 'uavList': ['auto'], 'values': [[{'lat': 28.144016372555452, 'lng': -16.503212302923206}, {'lat': 28.143671070700133, 'lng': -16.50311574339867}, {'lat': 28.143680531039788, 'lng': -16.50249615311623}, {'lat': 28.144023467787367, 'lng': -16.5025806427002}]]}, {'name': 'LandPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.143912309099793, 'lng': -16.50326326489449}}, {'name': 'LandPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_1'], 'values': {'lat': 28.143997451934666, 'lng': -16.50326862931252}}]}, 'from': 2, 'to': None}
        """
        
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
            self.mission_planner(self.client.mission_id, confirm, msg['payload'])
            # self.client.send_mission_info(
            #     self.mission_planner(self.client.mission_id, confirm, msg['payload'])
            # )
            
            # print("Send mission info")             
            # print(new_mission_info)
            # print()
            
            
            # print("Mission planner")
            # print(self.client.mission_id)
            # print(msg['payload']['uavList'])
            # print(msg['payload']['layers'])
        
        # self.mission_list.append(msg)
        
    def new_mission_callback(self, msg, args):
        print("AerostackUI - Confirm mission")
        #print(self)
        #print(msg)
        self.mission_interpreter(msg)
        
    def run(self):
        
        odom = []
        
        for uav in self.uav_id_list:
            odom.append([])
        
        while True:
            if (self.client.connection):
                incr = 0.0001
                for idx, uav in enumerate(self.uav_id_list):
                
                    # print(f"Uav List = {self.uav_id_list}")
                    # print(f"UAV = {uav}")
                    # print(f"Index = {idx}")
                    pose = [28.14376+incr, -16.50235+incr, 1, 0]
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
                    
                    odom[idx].append([pose[0], pose[1]])
                    
                    # print(f"Pose: {pose}")
                    # print(f"Orientation: {orientation}")
                    # print(f"Info: {info}")
                    # print(f"Odom: {odom[idx]}"")
                    
                    self.client.send_uav_info(
                        {
                            'id': str(uav),
                            'state': info, 
                            'pose': {'lat': pose[0], 'lng': pose[1], 'height': pose[2], 'yaw': orientation[2]},
                            'odom': odom[idx]
                        }
                    )
                    
                    incr += incr
                    
                    time.sleep(1)

        
if __name__ == '__main__':
    aerostackUI = AerostackUI()