import time
import threading
from pyrsistent import inc
import websocket
from websocket_client import WebSocketClient
import copy
import Unit_conversion as utm
import SwarmingLib as swarm
import numpy as np


class AerostackUI():
    def __init__(self):
        # self.client = WebSocketClient("ws://192.168.30.23:8000/ws/user/")
        self.client = WebSocketClient("ws://127.0.0.1:8000/ws/user/")

        self.client.addMsgCallback(
            'request', 'missionConfirm', self.new_mission_callback)
        self.client.addMsgCallback(
            'request', 'missionStart', self.start_mission_callback)
        self.client.addMsgCallback(
            'info', 'missionInfo', self.add_mission_callback)

        # self.drone_id = "drone_sim_rafa_0"  # "drone_sim_14"
        
        # self.uav_id_list = ['drone_sim_rafa_0']
        # self.uav_id_list_pos = [[28.14402, -16.50251, 0.0]]

        self.mission_list = {}
        self.speed = 2
        
        self.uav_id_list = [
            'M300',
            # 'drone_sim_8',
            # 'drone_sim_rafa_0',
        ]
        
        uav_id_mission = [
            'drone_sim_8',
            'drone_sim_rafa_0',
        ]

        self.uav_id_list_pos = [
            [28.1439717, -16.5032634, 0.0],
            [28.1438840, -16.5032570, 0.0],
        ]

        uav_id_list_last_pos = [
            [28.1439717, -16.5032634, 0.0],
            [28.1438460, -16.5032560, 0.0],
        ]
        
        streetSpacing = 8.0
        wpSpace = 4.0
        height = 10
        
        # n UAVs
        msg = {'type': 'request', 'header': 'missionConfirm', 'status': 'request', 'payload': {'status': 'request', 'id': 'New Mission', 'uavList': uav_id_mission, 'layers': []}, 'from': 2, 'to': None}
        
        for index, uav in enumerate(uav_id_mission):
            msg['payload']['layers'].append({'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': [uav], 'values': {'lat': self.uav_id_list_pos[index][0], 'lng': self.uav_id_list_pos[index][1]}})
        
        msg['payload']['layers'].append({'name': 'Area', 'height': [height, height], 'uavList': ['auto'], 'algorithm': 'Back and force', 'streetSpacing': streetSpacing, 'wpSpace': wpSpace, 'values': [[{'lat': 28.14400218209017, 'lng': -16.50320559740067}, {'lat': 28.143673435785125, 'lng': -16.503107696771625}, {'lat': 28.143685261209285, 'lng': -16.502482742071155}, {'lat': 28.144018737632805, 'lng': -16.50258600711823}]]})
        
        for index, uav in enumerate(uav_id_mission):
            msg['payload']['layers'].append({'name': 'LandPoint', 'height': [3, 3], 'uavList': [uav], 'values': {'lat': uav_id_list_last_pos[index][0], 'lng': uav_id_list_last_pos[index][1]}})
        
        
        
        time.sleep(1)

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

        time.sleep(2)
        
        # self.mission_interpreter(msg)

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
                {'name': 'TakeOffPoint', 'height': [
                    3, 3], 'values':  {'lat': 28.144, 'lng': -16.503}},
                {'name': 'Path', 'height': [3, 3],         'values': [{'lat': 28.144, 'lng': -16.503}, {'lat': 28.144, 'lng': -16.5026}, {'lat': 28.1435, 'lng': -16.5026},
                                                                      {'lat': 28.144, 'lng': -16.503}]},
                {'name': 'LandPoint', 'height': [3, 3],    'values':  {'lat': 28.144, 'lng': -16.503}}]
        }

        send_mission = {'status': 'confirmed', 'id': 1, 'uavList': ['drone_sim_rafa_0'], 'layers': [{'name': 'TakeOffPoint', 'height': [5, 5], 'values': {'lat': 28.1439938, 'lng': -16.5032927}}, {'name': 'Path', 'height': [5, 5], 'values': [{'lat': 28.143967888457993, 'lng': -16.503202915191654}, {'lat': 28.143970253536445, 'lng': -16.502602100372318}, {'lat': 28.143880380519278, 'lng': -16.502556502819065}, {'lat': 28.14387919797907, 'lng': -16.503158658742908}]}, {'name': 'WayPoint', 'height': [
            5, 5], 'values': {'lat': 28.143779864555484, 'lng': -16.50313183665276}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.14379287250906, 'lng': -16.50254845619202}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.143711277137808, 'lng': -16.502514928579334}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.143694721547664, 'lng': -16.503106355667118}}, {'name': 'LandPoint', 'height': [5, 5], 'values': {'lat': 28.14369708663212, 'lng': -16.503161340951923}}]}

        self.client.send_mission_info(send_mission)
        self.mission_planner(1, [self.drone_id],
                             [{'name': 'TakeOffPoint', 'height': [5, 5], 'values': {'lat': 28.1439938, 'lng': -16.5032927}}, {'name': 'Path', 'height': [5, 5], 'values': [{'lat': 28.143967888457993, 'lng': -16.503202915191654}, {'lat': 28.143970253536445, 'lng': -16.502602100372318}, {'lat': 28.143880380519278, 'lng': -16.502556502819065}, {'lat': 28.14387919797907, 'lng': -16.503158658742908}]}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.143779864555484,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                  'lng': -16.50313183665276}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.14379287250906, 'lng': -16.50254845619202}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.143711277137808, 'lng': -16.502514928579334}}, {'name': 'WayPoint', 'height': [5, 5], 'values': {'lat': 28.143694721547664, 'lng': -16.503106355667118}}, {'name': 'LandPoint', 'height': [5, 5], 'values': {'lat': 28.14369708663212, 'lng': -16.503161340951923}}]
                             )
        self.client.mission_id += 1

    def start_mission_callback(self, msg, args):
        print("AerostackUI - Start mission")
        print(msg)
        print(args)
        print(self.mission_list)
        """ 
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
        """

    
    def add_mission_callback(self, msg, args):
        print("AerostackUI - Add mission")
        print(msg)
        print(args)
        mission = msg['payload']
        
        id = mission['id']
        uavList = mission['uavList']
        status = mission['status']
        layers = mission['layers']
        
        for uav in uavList:
            if uav not in self.uav_id_list:
                print("Error: UAV not in list")
                return
        mission_layer = []
        for layer in layers:
            if (layer['type'] != 'Area'):
                mission_layer.append(layer)
        
        mission['layer'] = mission_layer
             
        self.mission_planner(id, status, mission)
        
        
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
                [initial_position[uav][0], initial_position[uav][1]]]

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
                [last_position[uav][0], last_position[uav][1]])

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
                self.mission_planner(self.client.mission_id,
                                     confirm, msg['payload'])
            )

    def new_mission_callback(self, msg, args):
        print("AerostackUI - Confirm mission")
        # print(self)
        # print(msg)
        self.mission_interpreter(msg)

    def run(self):

        odom = []

        for uav in self.uav_id_list:
            odom.append([])

        incr = 0.00000001
        while True:
            if (self.client.connection):
                
                for idx, uav in enumerate(self.uav_id_list):

                    # print(f"Uav List = {self.uav_id_list}")
                    # print(f"UAV = {uav}")
                    # print(f"Increment = {incr}")
                    # print(f"Index = {idx}")
                    pose = self.uav_id_list_pos[idx]
                    # pose_base = self.uav_id_list_pos[idx]
                    # pose = [pose_base[0] + incr, pose_base[1] + incr, pose_base[2] + incr]
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

                    print(f"Pose: {pose}")
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
                
                break
                    


if __name__ == '__main__':
    aerostackUI = AerostackUI()
