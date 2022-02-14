import websocket
import threading
import json
import time

def nullFunction(*args):
    """
    Default function for the websocket that does nothing
    """
    return


class UAV:
    """
    UAV object
    """
    def __init__(self, id, state, pose, odom=[], desired_path=[], sensors={}):
        self.id = id
        self.pose = pose
        self.state = state
        self.odom = odom
        self.desired_path = desired_path
        self.sensors = sensors
    
    def newPose(self, pose):
        """
        Add new UAV pose

        Args:
            pose (dict): UAV pose with format {'lat': x, 'lng': y, 'alt': h, 'yaw': theta}
        """
        self.pose = pose
        self.odom.append(pose)
        
    def newOdom(self, odom):
        """
        Set new UAV odometry with a list of poses

        Args:
            odom (list): List of poses with format {'lat': x, 'lng': y, 'alt': h, 'yaw': theta}
        """
        self.odom = odom
        
    def newDesiredPath(self, desired_path):
        """
        Set new desired path with a list of poses

        Args:
            desired_path (list): List of poses with format {'lat': x, 'lng': y, 'alt': h, 'yaw': theta}
        """
        self.desired_path = desired_path
        
    def newDesiredPose(self, pose):
        """
        Add new desired pose to the desired path

        Args:
            pose (dict): UAV pose with format {'lat': x, 'lng': y, 'alt': h, 'yaw': theta}
        """
        self.desired_path.append(pose)
        
    def newSensorList(self, sensor_list):
        self.sensor_list = sensor_list
        
    def newSensor(self, key, value):
        self.sensors[key] = value
        
    def getInfo(self):
        """
        Return UAV information in a dictionary

        Returns:
            [dict]: UAV information in a dictionary with format {'id': id, 'state': state, 'pose': pose, 'odom': odom, 'desired_path': desired_path}
        """
        return {
            'id': self.id,
            'state': self.state,
            'pose': self.pose,
            'odom': self.odom,
            'desiredPath': self.desired_path,
            'sensors': self.sensors
        }



# Asynchronous client to websockets server
class WebSocketClient:
    """
    Manager websocket client
    """
    def __init__(self, host="ws://127.0.0.1:8000/ws/user/"):
        self.host = host
        websocket.enableTrace(False)

        self.ws = websocket.WebSocketApp(self.host,
                                         on_open=self.onOpen,
                                         on_message=self.onMessage,
                                         on_error=self.onError,
                                         on_close=self.onClose)

        self.rol = 'manager'
        self.id = None
        
        self.on_message_function = None
        self.new_mission_function = None
        
        self.msg_id = 0
        self.mission_id = 1 # Can not be 0
        self.UAV_dict = {}
        self.missions_dict = {}

        # Execute run in a thread
        self.thread = threading.Thread(target=self.run)
        self.thread.start()


    def run(self):
        """
        Keep websocket open
        """
        print("Running")
        self.ws.run_forever()

    def close(self):
        """
        Close websocket connection
        """
        print("Closing connection")
        self.ws.close()
        self.thread.join()

    def onError(self, ws, error):
        """
        This function is called when websocket has an error
        """
        print(f"Error: {error}")
        self.ws.close()

    def onClose(self, ws, close_status_code, close_msg):
        """
        This function is called when websocket is closed
        """
        print("Connection closed")
        print(f"Status: {close_status_code}")
        print(f"Close message: {close_msg}")
        self.thread.join()

    def onOpen(self, ws):
        """
        This function is called when websocket is open
        """
        print("Connected")
        self.handshake()
        
    def onMessage(self, ws, message):
        """
        This function is called when websocket receives a message and manage it
        """
        print(f"Message recived: {message}")
        msg= json.loads(message)['message']
        
        # Communication with server
        if msg['type'] == 'basic' and msg['status'] == 'response':
            
            if msg['header'] == 'handshake':
                if msg['payload']['response'] == 'success':
                    print("Logged in")
                    self.id = msg['payload']['id']
                else:
                    print(f"Login failed with error {msg['response']}")
                    self.ws.close()
            
            elif msg['header'] == 'getId':
                self.id = msg['payload']['id']
            
            elif msg['header'] == 'ping':
                print(msg['payload'])
                
            elif msg['header'] == 'getClientList':
                print(msg['payload'])
                
        elif msg['type'] == 'request':
            
            if msg['header'] == 'confirmMission' and msg['status'] == 'request':
                self.new_mission_function(self, msg)
        
        else:
            print("Unknown message")
            print(msg)
            self.on_message_function(msg)
        
    def send(self, msg, to=None):
        """
        Send a JSON-decoded message to server

        Args:
            msg (dict): JSON-decoded message
            to (int, optional): Id of the destination client. Defaults to None.
        """
        msg['msg_id'] = self.msg_id
        self.msg_id+=1
            
        if self.id is not None:
            msg['id'] = self.id
            if to is None:
                msg['to'] = 0 # Server id
            else:
                msg['to'] = to
        
        message = json.dumps({'message': msg})
            
        self.ws.send('%s' % message)
        # print("Message sent: " + message)
    
    #region Basic communication
    
    def sendBasic(self, header, payload, to=None):
        """
        Send a basic message to server

        Args:
            header (str): Header of the message
            payload (dict): Payload of the message
            to (int, optional): Id of the destination client. Defaults to None.
        """
        msg = {
            'type': 'basic',
            'header': header,
            'status': 'request',
            'payload': payload
        }
        self.send(msg, to)
    
    def handshake(self):
        """
        Send handshake message to server
        """
        payload = {'rol': self.rol}
        self.sendBasic('handshake', payload)
    
    def getId(self):
        """
        Send getId message to server
        """        
        payload = {'id': self.id}
        self.sendBasic('getId', payload)
    
    def ping(self):
        """
        Send ping message to server
        """        
        payload = {}
        self.sendBasic('ping', payload)
        
    def getClientList(self):
        """
        Send getClientList message to server
        """        
        payload = {}
        self.sendBasic('getClientsList', payload)
        
    #endregion
    
    #region Info communication
    
    def sendInfo(self, header, payload, to=None):
        """
        Send a info message to server

        Args:
            header (str): Header of the message
            payload (dict): Payload of the message
            to (int, optional): Id of the destination client. Defaults to None.
        """
        msg = {
            'type': 'info',
            'header': header,
            'payload': payload
        }
        self.send(msg, to)
    
    def _sendUAVInfo(self, id):
        """
        Send UAV information to server by it id

        Args:
            id (int): Id of the UAV
        """
        payload = self.UAV_dict[id].getInfo()
        self.sendInfo('uavUpdate', payload)
        
    def newUAV(self, uav, send=True):
        """
        Add new UAV to the list with it state

        Args:
            uav (dict): UVA information with format {'id': id, 'state': state, 'pose': pose, 'odom': odom, 'desired_path': desired_path, 'sensors': sensors}
            send (bool, optional): Flag to send it to the server. Defaults to True.
        """
        id = uav['id']
        self.UAV_dict[id] = UAV(uav['id'], uav['state'], uav['pose'], uav['odom'], uav['desiredPath'], uav['sensors'])
        self._sendUAVInfo(id)

        
    def newUAVList(self, UAV_dict):
        """
        Add new UAV list to the list with it state

        Args:
            UAV_dict (dict): Dict with format {'id': id, 'state': state, 'pose': pose, 'odom': odom, 'desired_path': desired_path, 'sensors': sensors}
        """
        for uav in UAV_dict:
            self.newUAV(uav)
        
    def sendUavState(self, id, state):
        payload = {'id': id, 'state': state}
        self.sendInfo('uavState', payload)
        
    def sendUavPose(self, id, pose, to=None):
        payload = {'id': id, 'pose': pose}
        self.sendInfo('uavPose', payload, to)

    def sendUavDesiredPath(self, id, path, to=None):
        payload = {'id': id, 'desiredPath': path} 
        self.sendInfo('uavDesiredPath', payload)
        
    def sendUavOdom(self, id, odom, to=None):
        payload = {'id': id, 'odom': odom}
        self.sendInfo('uavOdom', payload, to)
        
    def sendUavSensors(self, id, sensors, to=None):
        payload = {'id': id, 'sensors': sensors}
        self.sendInfo('uavSensors', payload, to)
        
    #endregion
    
    #region Request communication
    def sendRequest(self, header, payload, to=None):
        """
        Send a request message to server

        Args:
            header (str): Header of the message
            payload (dict): Payload of the message
            to (int, optional): Id of the destination client. Defaults to None.
        """
        msg = {
            'type': 'request',
            'header': header,
            'status': 'response',
            'payload': payload
        }
        self.send(msg, to)
    
    def confirmMission(self, new_mission_id, status, layers, old_msg, extra=[]):
        payload = {
            'missionId': new_mission_id, 
            'status': status,
            'layers': layers,
            'oldMissionId': old_msg['payload']['missionId'],
            'extra': extra,
            'author': old_msg['id']
        }

        self.sendRequest('confirmMission', payload, to=old_msg['id'])
        self.missions_dict[mission_id] = payload
    
    
    #endregion


mission_id = 1

def newMissionCallback(client, msg):
    print(f"- Callback: Received: {msg}")
    confirm = True
    
    if msg["type"] == "request":
        if msg["header"] == "confirmMission":
            
            layers = msg["payload"]["layers"]
            
            for layer in layers:
                # If layer status is confirmed yet or 
                # layer Id is 0 -> New mission or
                # layer type is UAVMarker or UAVPath
                print(layer[4])
                if  not (layer[0] == 'draw' and layer[1] == 0 and (layer[4] == 'UAVMarker' or layer[4] == 'UAVPath' or layer[4] == 'UAVLandZone')):
                    confirm = False
            
            if confirm:
                       
                client.confirmMission(
                    client.mission_id,
                    'confirmed',
                    msg['payload']['layers'],
                    msg
                )
                
                client.mission_id += 1
                
            else:
                client.confirmMission(
                    0, # Implies mission is not valid
                    'denied',
                    msg['payload']['layers'],
                    msg,
                    extra='Some layers are not Marker or Path',
                )
                

def main():    
    
    client = WebSocketClient("ws://127.0.0.1:8000/ws/user/")
    print(client.mission_id)
    client.new_mission_function = newMissionCallback
    
    time.sleep(1)

    client.newUAVList([
        {'id': 'UAV 0', 'state': 'landed', 'pose': {'lat': 28.14376, 'lng': -16.50235, 'alt': 0, 'yaw': 0}, 'odom': [], 'desiredPath': [], 'sensors': {'battey': 80, 'temperature': 40}},
        {'id': 'UAV 1', 'state': 'landed', 'pose': {'lat': 28.14386, 'lng': -16.50245, 'alt': 0, 'yaw': 0}, 'odom': [], 'desiredPath': [], 'sensors': {'battey': 100, 'temperature': 20}} 
    ])
    

    time.sleep(1)
    
    lat = 28.14396
    lng = -16.50255
    incr = 0.000001
    
    odom = [
        [lat-incr*5, lng-incr*5],
        [lat, lng],
    ]
        
    desiredPath = [
        [lat, lng],
        [lat+incr*500, lng+incr*500],
        [lat+incr*1500, lng+incr*1000],
    ]
    
    client.newUAV(
        {'id': 'UAV 2', 'state': 'landed', 'pose': {'lat': lat, 'lng': lng, 'alt': 0, 'yaw': 0}, 'odom': odom, 'desiredPath': desiredPath, 'sensors': {'battey': 70, 'temperature': 60}}
    )
    
    
    while True:
        time.sleep(1. / 30)
        lat += incr
        lng += incr
        odom.append([lat, lng])
        client.sendUavPose('UAV 2', {'lat': lat, 'lng': lng, 'alt': 0, 'yaw': 0})
        client.sendUavOdom('UAV 2', odom)

    """
    time.sleep(2)
    client.sendUAVPose('UAV 0', 'fly', {'lat': 28.14406, 'lng': -16.50265, 'alt': 0, 'yaw': 0})
    
    
    
    client.newUAVList([
        {'id': 'UAV 0', 'state': 'fly', 'pose': {'lat': 0, 'lng': 0, 'alt': 0, 'yaw': 0}, 'odom': [], 'desiredPath': []},
        {'id': 'UAV 1', 'state': 'fly', 'pose': {'lat': 0, 'lng': 0, 'alt': 0, 'yaw': 0}, 'odom': [], 'desiredPath': []} 
    ])
    
    time.sleep(4)
    
    client.newUAVList([
        {'id': 'UAV 2', 'state': 'fly', 'pose': {'lat': 0, 'lng': 0, 'alt': 0, 'yaw': 0}, 'odom': [], 'desiredPath': []},
        {'id': 'UAV 3', 'state': 'fly', 'pose': {'lat': 0, 'lng': 0, 'alt': 0, 'yaw': 0}, 'odom': [], 'desiredPath': []} 
    ])
    """
    

if __name__ == "__main__":
    main()
