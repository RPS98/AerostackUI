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
sys.path.append(
    '/home/rafa/aerostack2_ws/src/aerostack2/stack/user_interface/python_interface/python_interface')

from drone_interface import DroneInterface

class UavInterface(DroneInterface):
    info_lock = threading.Lock()

    def __init__(self, uav_id, speed):
        self.drone_interface = super(UavInterface, self)
        self.drone_interface.__init__(uav_id)
        self.uav_id = uav_id
        # self.drone_interface = DroneInterface(uav_id)
        self.speed = speed

    def info_lock_decor(func):
        def wrapper(self, *args, **kwargs):
            with self.info_lock:
                return func(self, *args, **kwargs)
        return wrapper

    @info_lock_decor
    def get_info(self):
        pose = self.drone_interface.get_gps_pose()
        orientation = self.drone_interface.get_orientation()

        info_collection = {
            'id': self.drone_interface.get_drone_id(),
            'state': {},  # self.drone_interface.get_info(),
            'pose': {'lat': pose[0], 'lng': pose[1], 'height': pose[2], 'yaw': orientation[2]},
        }
        return info_collection

    def run_uav_mission(self, mission, thread):
        print("run_uav_mission")
        print(mission)

        uav = self.uav_id
        drone_interface = self.drone_interface

        for element in mission:
            # print("Element: ", element)

            if element['name'] == 'TakeOffPoint':

                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]

                print(f"{uav} - Send takeoff")
                print(waypoint[2])
                drone_interface.takeoff(height=waypoint[2])
                print(f"{uav} - Takeoff done")

            elif element['name'] == 'LandPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]

                print("Send land point: {waypoint}")
                drone_interface.follow_gps_path([waypoint])

                print("Send land")
                drone_interface.land()
                print(f"Land done")

            elif element['name'] == 'Path':
                waypoint = element['values']

                print(f"{uav} - Send path")
                # print(f"Send path: {waypoint}")
                drone_interface.follow_gps_path(waypoint, self.speed)

            elif element['name'] == 'WayPoint':
                waypoint = [
                    element['values'][0][0],
                    element['values'][0][1],
                    element['values'][0][2]
                ]
                print(f"Send waypoint: {waypoint}")
                drone_interface.follow_gps_wp([waypoint], self.speed)

            elif element['name'] == 'Area':
                waypoint = element['values']
                print(f"{uav} - Send area")
                # print(f"Send area: {waypoint[1:]}")
                drone_interface.follow_gps_path(waypoint[1:], self.speed)
                print(f"Area sent")

            else:
                print("Unknown layer")
                print("Element: ", element)
                raise Exception(
                    "Unknown mission element name: ", element['name'])

        if thread != None:
            thread.join()


class MissionManager():
    def __init__(self):
        self.mission_id = 0
        self.mission_list = {}

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

        # TODO: Check if mission mission_id change
        confirm_msg = {
            'id': self.mission_id,
            'status': confirm,
            'extra': extra
        }

        self.mission_id += 1

        return confirm_msg

    def mission_planner(self, mission_id, mission_info):
        """Convert Aerostack UI mission to ROS mission

        Args:
            mission_id (number): Mission id
            uavList (list): List of UAVs
            layers (list): List of layers
        """

        # print("AerostackUI - Mission planner")
        # print(mission_id)
        # print(mission_info)

        send_mission = {
            'id': mission_id,
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


class AerostackUI():
    def __init__(self, uav_id_list):
        # ws://127.0.0.1:8000/ws/user/") #"ws://192.168.30.23:8000/ws/user/")
        self.client = WebSocketClient("ws://127.0.0.1:8000/ws/user/")

        self.mission_manager = MissionManager()

        self.client.addMsgCallback(
            'request', 'missionConfirm', self.mission_confirm_callback)
        self.client.addMsgCallback(
            'request', 'missionStart', self.start_mission_callback)

        self.uav_id_list = uav_id_list

        self.speed = 6

        self.executor = rclpy.executors.MultiThreadedExecutor()

        self.drone_interface = {}
        for uav_id in self.uav_id_list:
            # self.drone_interface[uav_id] = DroneInterface(uav_id)
            # if uav_id == 'drone_sim_8':
            #     self.drone_interface[uav_id] = None
            # else:
            drone_node = UavInterface(uav_id, self.speed)
            self.executor.add_node(drone_node)
            self.drone_interface[uav_id] = drone_node

        self.keep_running = True
        self.spin_thread = threading.Thread(
            target=self.auto_spin, daemon=False)
        self.spin_thread.start()

        # executor_thread = threading.Thread(target=executor.spin, daemon=True)
        # executor_thread.start()

        self.uav_id_list_pos = [
            [28.1439717, -16.5032634, 0.0],
            [28.1438840, -16.5032570, 0.0],
        ]

        time.sleep(3)

        self.get_info_thread = threading.Thread(target=self.run)
        self.get_info_thread.start()

        # self.fake_mission()

    def auto_spin(self):
        while rclpy.ok() and self.keep_running:
            # rclpy.spin_once(self)
            self.executor.spin()
            time.sleep(0.1)

    def fake_mission(self):
        uav_id_mission = [
            'drone_sim_rafa_0',
            'drone_sim_8',
        ]

        uav_id_list_last_pos = [
            [28.1439717, -16.5032634, 0.0],
            [28.1438460, -16.5032560, 0.0],
        ]

        streetSpacing = 8.0
        wpSpace = 1.0
        height = 10

        # n UAVs
        # msg = {'type': 'request', 'header': 'missionConfirm', 'status': 'request', 'payload': {'status': 'request', 'id': 'New Mission', 'uavList': uav_id_mission, 'layers': []}, 'from': 2, 'to': None}

        # for index, uav in enumerate(uav_id_mission):
        #     msg['payload']['layers'].append({'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': [uav], 'values': {'lat': self.uav_id_list_pos[index][0], 'lng': self.uav_id_list_pos[index][1]}})

        # msg['payload']['layers'].append({'name': 'Area', 'height': [height, height], 'uavList': ['auto'], 'algorithm': 'Back and force', 'streetSpacing': streetSpacing, 'wpSpace': wpSpace, 'values': [[{'lat': 28.14400218209017, 'lng': -16.50320559740067}, {'lat': 28.143673435785125, 'lng': -16.503107696771625}, {'lat': 28.143685261209285, 'lng': -16.502482742071155}, {'lat': 28.144018737632805, 'lng': -16.50258600711823}]]})

        # for index, uav in enumerate(uav_id_mission):
        #     msg['payload']['layers'].append({'name': 'LandPoint', 'height': [3, 3], 'uavList': [uav], 'values': {'lat': uav_id_list_last_pos[index][0], 'lng': uav_id_list_last_pos[index][1]}})

        # msg = {'type': 'request', 'header': 'missionConfirm', 'status': 'request', 'payload': {'status': 'request', 'id': 'New Mission', 'uavList': ['drone_sim_rafa_0', 'drone_sim_8'], 'layers': [{'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.1439711, 'lng': -16.5032646}}, {'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': ['drone_sim_8'], 'values': {'lat': 28.1439709, 'lng': -16.503213}}, {'name': 'Area', 'height': [10, 10], 'uavList': ['auto'], 'algorithm': 'Back and force', 'streetSpacing': 5, 'wpSpace': 1, 'values': [[{'lat': 28.144008094784244, 'lng': -16.503208279609684}, {'lat': 28.143668705615095, 'lng': -16.503123790025715}, {'lat': 28.143676983412508, 'lng': -16.502502858638767}, {'lat': 28.144027015403147, 'lng': -16.5025806427002}]]}, {'name': 'LandPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.1439711, 'lng': -16.5032646}}, {'name': 'LandPoint', 'height': [3, 3], 'uavList': ['drone_sim_8'], 'values': {'lat': 28.143680531039788, 'lng': -16.50316268205643}}]}, 'from': 3, 'to': None}

        msg = {'type': 'request', 'header': 'missionConfirm', 'status': 'request', 'payload': {'status': 'request', 'id': 'New Mission', 'uavList': ['drone_sim_rafa_0', 'drone_sim_rafa_1'], 'layers': [{'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.1439711, 'lng': -16.5032644}}, {'name': 'TakeOffPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_1'], 'values': {'lat': 28.1439711, 'lng': -16.5032645}}, {'name': 'Area', 'height': [10, 10], 'uavList': ['auto'], 'algorithm': 'Back and force', 'streetSpacing': 4, 'wpSpace': 1, 'values': [[{'lat': 28.14400572970666, 'lng': -16.503208279609684}, {'lat': 28.143673435785125, 'lng': -16.503118425607685}, {'lat': 28.143685261209285, 'lng': -16.5024907886982}, {'lat': 28.14401519001672, 'lng': -16.502592712640766}]]}, {'name': 'LandPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_0'], 'values': {'lat': 28.143902848780623, 'lng': -16.503228396177295}}, {'name': 'LandPoint', 'height': [3, 3], 'uavList': ['drone_sim_rafa_1'], 'values': {'lat': 28.143673435785125, 'lng': -16.503165364265445}}]}, 'from': 2, 'to': None}

        time.sleep(3)
        self.mission_confirm_callback(msg)

    def mission_confirm_callback(self, msg, args):
        confirm_msg = self.mission_manager.mission_interpreter(msg)

        self.client.missionConfirm(
            confirm_msg['id'],
            confirm_msg['status'],
            msg['payload']['id'],
            msg['from'],
            confirm_msg['extra']
        )

        if confirm_msg['status'] == 'confirmed':
            # self.mission_planner(self.client.mission_id, confirm, msg['payload'])

            mission_planner_msg = self.mission_manager.mission_planner(
                str(confirm_msg['id']),
                msg['payload']
            )

            mission_planner_msg['status'] = confirm_msg['status']

            self.client.send_mission_info(mission_planner_msg)

    def start_mission_callback(self, msg, args):
        print("AerostackUI - Start mission")
        print(msg)

        mission_id = str(msg['payload']['id'])
        mission_list = self.mission_manager.mission_list[mission_id]

        print("- Start mission ", mission_id)
        self.thread_uav = {}
        for uav in mission_list:
            drone_interface = self.drone_interface[uav]

            if drone_interface == None:
                continue

            print("Starting mission for uav ", uav)
            self.thread_uav[uav] = None
            mission_for_uav = mission_list[uav]

            self.thread_uav[uav] = threading.Thread(target=drone_interface.run_uav_mission, args=[
                                                    mission_for_uav, self.thread_uav[uav]])
            self.thread_uav[uav].start()

    def run(self):

        odom = {}

        for uav in self.uav_id_list:
            odom[uav] = []

        while self.client.connection:

            for idx, uav in enumerate(self.uav_id_list):
                drone_interface_i = self.drone_interface[uav]

                if drone_interface_i == None:
                    pose = self.uav_id_list_pos[idx]

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

                    send_info = {
                        'id': str(uav),
                        'state': info,
                        'pose': {'lat': pose[0], 'lng': pose[1], 'height': pose[2], 'yaw': orientation[2]},
                        'odom': odom[uav]
                    }

                else:

                    send_info = drone_interface_i.get_info()
                    # print(f"Send info for {uav}")
                    # print(send_info)

                odom[uav].append(
                    [send_info['pose']['lat'], send_info['pose']['lng']])
                send_info['odom'] = odom[uav]

                pose_fail = {'lat': 0.0, 'lng': 0.0}
                
                if send_info['pose']['lat'] != pose_fail['lat'] or send_info['pose']['lng'] != pose_fail['lng']:
                    self.client.send_uav_info(send_info)
                else:
                    print("Error sending info for ", uav)
                    print(send_info['pose'])
                    odom[uav] = []

            time.sleep(1)
            # else:
            #     print("Conecction lost")
            #     time.sleep(1)

        self.get_info_thread.join()


if __name__ == '__main__':
    rclpy.init()
    # uav_list = [
    #     'drone_sim_rafa_0',
    #     'drone_sim_8',
    # ]
    uav_list = [
        'M200',
        'M300',
    ]
    aerostackUI = AerostackUI(uav_list)
