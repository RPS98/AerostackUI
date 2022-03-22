from turtle import position
import rclpy
import threading
import time 

import websocket
from websocket_client import WebSocketClient
import copy
import Unit_conversion as utm
import SwarmingLib as swarm
import numpy as np

import sys
sys.path.append('/home/rafa/aerostack2_ws/src/aerostack2/stack/user_interface/python_interface/python_interface')

from drone_interface import DroneInterface

class AerostackUI():
    def __init__(self):
        self.client = WebSocketClient("ws://127.0.0.1:8000/ws/user/") #ws://127.0.0.1:8000/ws/user/") #"ws://192.168.30.23:8000/ws/user/")
        self.client.addMsgCallback('request', 'missionConfirm', self.new_mission_callback)
        self.client.addMsgCallback('request', 'missionStart', self.start_mission_callback)
        
        
        rclpy.init()
        
        self.drone_id = "drone_sim_rafa_0" # "drone_sim_14"
        self.drone_interface = DroneInterface(self.drone_id)
        self.mission_list = {}
        self.speed = 6
        
        time.sleep(1)
        
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        
        # 1 UAV
        time.sleep(4)
        msg = {'type': 'request', 'header': 'missionConfirm', 'status': 'request', 'payload': {'status': 'request', 'id': 'New Mission', 'uavList': ['drone_sim_rafa_0'], 'layers': [{'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.1439951, 'lng': -16.5025275}}, {'name': 'Area', 'height': [10, 10], 'uavList': ['auto'], 'algorithm': 'Back and force', 'streetSpacing': 4, 'wpSpace': 1, 'values': [[{'lat': 28.144050666172078, 'lng': -16.50243848562241}, {'lat': 28.143798785214713, 'lng': -16.50237008929253}, {'lat': 28.143831896360293, 'lng': -16.501101404428486}, {'lat': 28.14408495977774, 'lng': -16.501352190971378}]]}, {'name': 'LandPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.14380351537897, 'lng': -16.50243848562241}}]}, 'from': 2, 'to': None}

        self.mission_interpreter(msg)
        
    def run_uav_mission(self, mission):
        for element in mission:
            # print("Element: ", element)
            
            if element['name'] == 'TakeOffPoint':
                
                waypoint =[
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                
                print(f"Send takeoff")
                print(waypoint[2])
                self.drone_interface.takeoff(height=waypoint[2])
                print(f"Takeoff done")
            
            elif element['name'] == 'LandPoint':
                waypoint =[
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]

                print("Send land point: {waypoint}")
                self.drone_interface.follow_gps_path([waypoint])

                print("Send land")
                self.drone_interface.land()
                print(f"Land done")
                
            elif element['name'] == 'Path':
                waypoint = element['values']
                print(f"Send path: {waypoint}")
                self.drone_interface.follow_gps_path(waypoint, self.speed)
                
            elif element['name'] == 'WayPoint':
                waypoint =[
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                print(f"Send waypoint: {waypoint}")
                self.drone_interface.follow_gps_wp([waypoint], self.speed)
            
            elif element['name'] == 'Area':
                waypoint = element['values']
                print(f"Send area")
                # print(f"Send area: {waypoint}")
                self.drone_interface.follow_gps_path(waypoint, self.speed)
            
            else:
                print("Unknown layer")
                print("Element: ", element)
                raise Exception("Unknown mission element name: ", element['name'])
            
                
    def start_mission_callback(self, msg, args):
        print("AerostackUI - Start mission")
        print(msg)
        # print(args)
        # print(self.mission_list)
        
        mission_id = str(msg['payload']['id'])
        mission_list = self.mission_list[mission_id]
        
        print("- Start mission ", mission_id)
        self.thread_uav = {}
        for uav in mission_list:
            print("Starting mission for uav ", uav)
            # print(self.mission_list[mission_id][uav])
            mission_layers = self.mission_list[mission_id][uav]
            self.thread_uav[uav] = threading.Thread(target=self.run_uav_mission, args=[mission_layers])
            
            self.thread_uav[uav].start()
                                   
    def swarm_planning(self, uavList, initial_position, last_position, height, values, algorithm, streetSpacing, wpSpace):

        zone = None
        letter = None

        # Convert to utm
        initial_position_utm = {}
        for uav in uavList:
            east, north, zone_number, zone_letter = utm.GPS_to_UTM(
                initial_position[uav][0], initial_position[uav][1])
            initial_position_utm[uav] = [east, north]
            zone = zone_number
            letter = zone_letter

        last_position_utm = {}
        for uav in uavList:
            east, north, zone_number, zone_letter = utm.GPS_to_UTM(
                last_position[uav][0], last_position[uav][1])
            last_position_utm[uav] = [east, north]

        values_utm = []
        for value in values:
            east, north, zone_number, zone_letter = utm.GPS_to_UTM(
                value['lat'], value['lng'])
            values_utm.append([east, north])

        # Swarm planning
        UAV_initial_position = []
        for uav in initial_position_utm:
            UAV_initial_position.append(
                [initial_position_utm[uav][0], initial_position_utm[uav][1]])

        UAV_last_position = []
        for uav in last_position_utm:
            UAV_last_position.append(
                [last_position_utm[uav][0], last_position_utm[uav][1]])

        vel_input = np.full(len(UAV_initial_position), 1)
        vel_sum = sum(vel_input)
        uav_weight = np.zeros_like(vel_input, dtype=float)
        for i in range(0, len(vel_input)):
            uav_weight[i] = float(vel_input[i])/vel_sum

        waypoints, wpt_grid = swarm.compute_area(
            np.array(UAV_initial_position),
            np.array(UAV_last_position),
            uav_weight,
            np.array(values_utm),
            altitude=height,
            street_spacing=streetSpacing,
            wpt_separation=wpSpace,
            path_algorithm=algorithm,
            distribution_algorithm="binpat",
        )

        # Data format
        uavList_wp_utm = {}
        for index, uav_wp in enumerate(waypoints):
            uav_wp_aux = []
            for wp in uav_wp:
                if not np.isnan(wp[0]) or not np.isnan(wp[1]):
                    uav_wp_aux.append(wp)
                else:
                    break
            uavList_wp_utm[uavList[index]] = uav_wp_aux

        uavList_wp_gps = {}
        uavList_wp_gps_v2 = {}

        for uav in uavList:

            uavList_wp_gps[uav] = [
                {'lat': initial_position[uav][0], 'lng': initial_position[uav][1]}]
            uavList_wp_gps_v2[uav] = [
                [initial_position[uav][0], initial_position[uav][1], height]]

            utm_values = uavList_wp_utm[uav]
            for utm_value in utm_values:
                gps_value = utm.UTM_to_GPS(
                    utm_value[0], utm_value[1], zone, letter)
                uavList_wp_gps[uav].append(
                    {'lat': gps_value[0], 'lng': gps_value[1]})
                uavList_wp_gps_v2[uav].append(
                    [gps_value[0], gps_value[1], height])

            uavList_wp_gps[uav].append(
                {'lat': last_position[uav][0], 'lng': last_position[uav][1]})
            uavList_wp_gps_v2[uav].append(
                [last_position[uav][0], last_position[uav][1], height])

        return uavList_wp_gps, uavList_wp_gps_v2

    def get_next_position(self, sublist, mission, last_position, first_uav, mission_info):
        mission_aux = copy.deepcopy(mission)
        last_position_aux = copy.deepcopy(last_position)

        last_position_list = {}
        last_position_flag = {}
        for uav in mission_info['uavList']:
            last_position_list[str(uav)] = [None, None, None]
            last_position_flag[str(uav)] = False

        for layer in sublist:

            send_layer, mission, last_position = self.layer_interpreter(
                layer, mission_aux, last_position_aux, first_uav, mission_info)

            for uav in last_position:
                if last_position_flag[uav] == False:
                    last_position_list[uav] = last_position[uav]
                    last_position_flag[uav] = True

            all_flags = True
            for uav in last_position_flag:
                if not last_position_flag[uav]:
                    all_flags = False

            if all_flags:
                return last_position_list

        raise Exception("Next position for UAV not found")

    def layer_interpreter(self, layer, mission, last_position, first_uav, mission_info):
        name = layer['name']
        uavList = layer['uavList']
        height = layer['height']
        values = layer['values']

        send_layer = {
            'name': name,
            'uavList': uavList,
            'height': height,
            'values': values
        }

        new_last_position = {}

        if (name == 'TakeOffPoint' or name == 'LandPoint' or name == 'WayPoint'):
            uav = uavList[0]
            if uav == 'auto':
                uav = first_uav

            marker_position = [values['lat'], values['lng'], height[1]]
            new_last_position[uav] = marker_position

            mission[uav].append({
                'name': name,
                'values': [marker_position]
            })
            send_layer['uavList'] = [uav]

        elif (name == 'Path'):
            uav = uavList[0]
            if uav == 'auto':
                uav = first_uav

            waypoints = []
            for point in layer['values']:
                waypoints.append(
                    [point['lat'], point['lng'], layer['height'][1]])

            new_last_position[uav] = waypoints[len(waypoints)-1]

            mission[uav].append({
                'name': name,
                'values': waypoints
            })

            send_layer['uavList'] = [uav]

        elif name == 'Area':

            if uavList[0] == 'auto':
                uavListAux = mission_info['uavList']
            else:
                uavListAux = uavList

            index = mission_info['layers'].index(layer)
            range_to_end = range(index+1, len(mission_info['layers']))
            sublist = [mission_info['layers'][i] for i in range_to_end]

            algorithm = layer['algorithm']
            streetSpacing = layer['streetSpacing']
            wpSpace = layer['wpSpace']

            next_position = self.get_next_position(
                sublist, mission, last_position, first_uav, mission_info)

            uavPath, path = self.swarm_planning(
                uavListAux, last_position, next_position, height[1], values[0], algorithm, streetSpacing, wpSpace)

            send_layer['uavPath'] = {}
            for uav in uavListAux:
                new_last_position[uav] = next_position[uav]
                mission[uav].append({
                    'name': name,
                    'values': path[uav]
                })

            send_layer['uavPath'] = uavPath
            send_layer['uavList'] = uavListAux

        else:
            raise Exception("Unknown layer name")

        return send_layer, mission, new_last_position

    def mission_planner(self, mission_id, confirm, mission_info):
        """Convert Aerostack UI mission to ROS mission

        Args:
            mission_id (number): Mission id
            uavList (list): List of UAVs
            layers (list): List of layers
        """

        # print("AerostackUI - Mission planner")
        # print(mission_id)
        # print(confirm)
        # print(mission_info)

        send_mission = {
            'id': mission_id,
            'status': confirm,
            'uavList': mission_info['uavList'],
            'layers': []
        }

        first_uav = mission_info['uavList'][0]

        mission = {}
        last_position = {}
        for uav in mission_info['uavList']:
            mission[str(uav)] = []
            last_position[str(uav)] = [None, None, None]

        for layer in mission_info['layers']:

            send_layer, mission, new_last_position = self.layer_interpreter(
                layer, mission, last_position, first_uav, mission_info)

            for uav in new_last_position:
                last_position[uav] = new_last_position[uav]

            send_mission['layers'].append(send_layer)

        self.mission_list[mission_id] = mission
        return send_mission
    
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
                extra.append(
                    'Invalid id, only "New Mission" is allowed for now :)')

        self.client.missionConfirm(
            self.client.mission_id,
            confirm,
            msg['payload']['id'],
            msg['from'],
            extra
        )

        self.client.mission_id += 1

        if confirm == 'confirmed':
            # self.mission_planner(self.client.mission_id, confirm, msg['payload'])
            self.client.send_mission_info(
                self.mission_planner(str(self.client.mission_id),
                                     confirm, msg['payload'])
            )
        
    def new_mission_callback(self, msg, args):
        print("AerostackUI - Confirm mission")
        #print(self)
        #print(msg)
        self.mission_interpreter(msg)
        
    def run(self):
        
        odom = []
        
        
        while rclpy.ok():
            if (self.client.connection):
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
                        'id': str(self.drone_id),
                        'state': info, 
                        'pose': {'lat': pose[0], 'lng': pose[1], 'height': pose[2], 'yaw': orientation[2]},
                        'odom': odom 
                    }
                )
                
                time.sleep(0.1)
            else:
                print("Conecction lost")
                time.sleep(1)
                
        
        
if __name__ == '__main__':
    aerostackUI = AerostackUI()