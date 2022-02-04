import json
from channels.generic.websocket import AsyncJsonWebsocketConsumer


class ServerManager():
    """
    Server manager that handles all clients connections and messages
    """

    def __init__(self):
        # List of clients connected to the server and its role
        self.client_list = []   # Client list is [id, rol , consumer]
        self.manager_list = []  # Manager list is [consumer]
        self.web_page_list = [] # Webpage list is [id, consumer]
        self.client_id = 2      # Client id=0 is the server. Client id=1 is the manager. Client id=2 is the first webpage client.   
        self.mission_id = 1     # Mission id=0 is a new mission. Mission id=1 is the first confirmed mission.
        self.missions_dict = {} # Missions dict is {mission_id: mission_data}
        self.UAV_list = {}      # UAV list is {id: {'id': id, 'state': state, 'pose': pose, 'odom': odom, 'desired_path': desired_path}}
        
    async def clientDisconnect(self, id, rol):
        """
        Remove client from client list and from manager list or webpage list

        Args:
            id (int): Client id
            rol (str): Client rol: 'manager' or 'webpage'
        """
        for idx, val in enumerate(self.client_list):
            if val[0] == id:
                self.client_list.pop(idx)
                break
        
        if rol == 'manager':
            for idx, val in enumerate(self.manager_list):
                if val[0] == id:
                    self.manager_list.pop(idx)
                    break
        
        elif rol == 'webpage':
            for idx, val in enumerate(self.web_page_list):
                if val[0] == id:
                    self.web_page_list.pop(idx)
                    break
                
    async def sendMessage(self, sender, receiver, msg):
        """
        Send message recived from the "sender" to the "receiver"

        Args:
            sender (int): Id of the sender
            receiver (int): Id of the receiver
            msg (dict): Message to be sent in json format
        """
        if receiver == 'broadcast':
            for client in self.client_list:
                if client[0] != sender:
                    await client[2].sendMessage(msg)
                    
        elif receiver in range(0, self.client_id):
            for client in self.client_list:
                if client[0] == receiver:
                    await client[2].sendMessage(msg)
                    break

        elif receiver == 'webpage':
            for wp in self.web_page_list:
                if wp[0] != sender:
                    await wp[1].sendMessage(msg)
        
        elif receiver == 'manager':
            for manager in self.manager_list:
                await manager[0].sendMessage(msg) 
    
    #region Basic communication
    def newWebpage(self, consumer):
        """
        Append webpage consumer to client and webpages lists, and assign an id

        Args:
            consumer (object): CLientWebsocket instance

        Returns:
            id (int): Id assigned to the webpage
        """
        id = self.client_id
        self.client_list.append([id, 'webpage' , consumer])
        self.web_page_list.append([id, consumer])
        self.client_id += 1
        return id
    
    def newManager(self, consumer):
        """
        Append manager consumer to client and manager lists, and assign an id = 1

        Args:
            consumer (object): CLientWebsocket instance

        Returns:
            id (int): Id assigned to the client
        """
        self.client_list.append([1, 'manager', consumer])
        self.manager_list.append([consumer])
        return 1

    def getClientList(self):
        """
        Return the client list with the id and rol
        """
        client_list = []
        for client in self.client_list:
            client_list.append([client[0], client[1]])
        return client_list
    #endregion
    
    #region Info communication
    async def sendUAVList(self):
        """
        Send the UAV list to all Webpages
        """
        payload = {
            'UAVList': self.UAV_list
        }
        msg = {
            'type': 'info',
            'header': 'newUAVList',
            'payload': payload
        }
        await self.sendMessage(0, 'webpage', msg)
        
    async def sendUAV(self, id):
        """
        Send the new UAV to all Webpages
        """
        payload = {
            'UAV': self.UAV_list[id]
        }
        msg = {
            'type': 'info',
            'header': 'newUAV',
            'payload': payload
        }
        await self.sendMessage(0, 'webpage', msg)
        
        
    async def newUAV(self, msg):
        """
        Add new UAV to the UAV list

        Args:
            msg (dict): Message with the UAV data with format {'id': id, 'state': state, 'pose': pose, 'odom': odom, 'desired_path': desired_path}
        """
        self.UAV_list[msg['payload']['UAV']['id']] = msg['payload']['UAV']
        await self.sendUAV(msg['payload']['UAV']['id'])
        
    async def newUAVList(self, msg):
        """
        Replace the UAV list for a new one

        Args:
            msg (dict): Message with a list of UAVs data with format {'id': id, 'state': state, 'pose': pose, 'odom': odom, 'desired_path': desired_path}
        """
        UAV_list_aux = msg['payload']['UAVList']
        for uav in UAV_list_aux:
            self.UAV_list[uav['id']] = uav
        await self.sendUAVList()
        
    async def UAVPose(self, msg):
        
        print("In UAVPose")
        print()
        print(self.UAV_list)
        print()
        print(msg)
        
        self.UAV_list[msg['payload']['UAVId']]['state'] = msg['payload']['state']
        self.UAV_list[msg['payload']['UAVId']]['pose']  = msg['payload']['pose']
        await self.sendMessage(0, 'webpage', msg)
        
    async def UAVOdom(self, msg):
        self.UAV_list[msg['payload']['UAVId']]['odom'] = msg['payload']['odom']
        await self.sendMessage(0, 'webpage', msg)
        
    async def UAVDesiredPath(self, msg):
        self.UAV_list[msg['payload']['UAVId']]['desired_path'] = msg['payload']['desired_path']
        await self.sendMessage(0, 'webpage', msg)
        
    async def UAVSensors(self, msg):
        self.UAV_list[msg['payload']['UAVId']]['sensors'] = msg['payload']['sensors']
        await self.sendMessage(0, 'webpage', msg)
        
    #endregion
    
    #region Request communication
    async def confirmMissionRequest(self, msg):
        await self.sendMessage(0, 'manager', msg)
        
    async def confirmMissionResponse(self, msg):
        if msg['payload']['status'] == 'confirmed':
            
            mission_recived = {
                'missionId': msg['payload']['missionId'],
                'status': msg['payload']['status'],
                'layers': msg['payload']['layers'],
            }
            
            self.missions_dict[msg['payload']['missionId']] =  mission_recived
            
            await self.sendMessage(0, 'webpage', msg)
        elif msg['payload']['status'] == 'denied':
            await self.sendMessage(0, msg['to'], msg)
    
    #endregion   
        
serverManager = ServerManager()


class CLientWebsocket(AsyncJsonWebsocketConsumer):
    """
     It handle the client connection and communication

    Args:
        AsyncJsonWebsocketConsumer (object): Variant of AsyncWebsocketConsumer that automatically JSON-encodes and decodes
        messages as they come in and go out.
    """
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.group_name = [] # May be webpage or broadcast
        self.rol = None # Consumer role (webpage or manager)
        self.identified = False # Flag to identify if the client is identified
        self.id = None

    async def connect(self):
        """
        Accepts an incoming socket
        """
        print("Connected")
        await self.accept()

    async def disconnect(self, close_code):
        """
        Called when a WebSocket connection is closed.
        """
        print(f"Disconnected {self.rol} with id: {self.id}")
        serverManager.clientDisconnect(self.id, self.rol)
        for name in self.group_name:
            await self.channel_layer.group_discard(
                name,
                self.channel_name
            )

    async def receive(self, text_data):
        """
        Called with a decoded WebSocket frame.

        Args:
            message (str): JSON-decoded incoming message
        """
        
        print(f"Received message: {text_data}")        
        msg = json.loads(text_data)['message']
        
        if msg['type'] == 'basic' and msg['status'] == 'request':
            
            if msg['header'] == 'handshake':
                await self._handshake(msg['payload']['rol'])
            
            elif msg['header'] == 'getId':
                await self._getId()
                
            elif msg['header'] == 'ping':
                await self._ping()
            
            elif msg['header'] == 'getClientsList':
                await self._getClientsList()
                
        elif self.identified:
            if msg['type'] == 'info':
                if msg['header'] == 'newUAV':
                    await serverManager.newUAV(msg)
                    
                elif msg['header'] == 'newUAVList':
                    await serverManager.newUAVList(msg)
                    
                elif msg['header'] == 'UAVPose':
                    await serverManager.UAVPose(msg)
                    
                elif msg['header'] == 'UAVOdom':
                    await serverManager.UAVOdom(msg)
                    
                elif msg['header'] == 'UAVDesiredPath':
                    await serverManager.UAVDesiredPath(msg)
                    
                elif msg['header'] == 'UAVSensors':
                    await serverManager.UAVSensors(msg)
                    
            elif msg['type'] == 'request':
                if msg['header'] == 'getUAVList':
                    await self._getUAVList()
                    
                elif msg['header'] == 'getMissionList':
                    await self._getMissionList()
                    
                elif msg['header'] == 'confirmMission':
                    if msg['status'] == 'request':
                        await serverManager.confirmMissionRequest(msg)

                    elif msg['status'] == 'response':
                        await serverManager.confirmMissionResponse(msg)
                    
            else:
                # TODO: send msg to destinatary
                pass
            
        print("Message processed")
        
         
    @staticmethod        
    async def checkMessage(message):
        """Check if incoming message is valid

        Args:
            message (str): JSON-decoded message

        Returns:
            bool: True if the message is valid, False otherwise
        """
        if ((message['header'] is not None) and 
            (message['type'] is  not None)
            ):
            return True
        else:
            return False
        
         
    async def addToGroup(self, group_name):
        """
        Add client to a group for send and receive messages

        Args:
            group_name (srt): Group name
        """
        self.group_name.append(group_name)
        # Join room group
        await self.channel_layer.group_add(
            group_name,
            self.channel_name
        )
        
    async def sendMessage(self, msg, group_name=None):
        """
        Send message to the client

        Args:
            msg (str): JSON-decoded message to be sent
            group_name (str, optional): Group name. Defaults to None.
        """
        if group_name is None:
            print(f"-{self.rol} {self.id}: Send message to channel: {msg}")
            await self.send_json({
                'message': msg
            })
        else:
            print(f"-{self.rol} {self.id}: Send message to group {group_name}: {msg}")
            await self.channel_layer.group_send(
                group_name,
                {
                    'type': 'groupMessage',
                    'message': msg
                }
            )

    async def groupMessage(self, message):
        """
        Receive message from group

        Args:
            message (str): JSON-decoded incoming message
        """
        msg = message['message']

        # Send message to WebSocket
        await self.send(msg=json.dumps({
            'message': msg
        }))

    #region Basic communication
    async def _handshake(self, rol):
        """
        Accept the client's handshake
        
        Args:
            rol (str): Client rol (webpage or manager)
        """
        self.identified = True
        self.rol = rol
        
        if self.rol == 'webpage':
            # Get an identifier
            self.id = serverManager.newWebpage(self)

            payload = {
                'response': 'success',
                'id': self.id
            }
            response = {
                'type': 'basic',
                'header': 'handshake',
                'status': 'response',
                'payload': payload
            }
            
            await self.addToGroup(group_name='broadcast')
            await self.addToGroup(group_name='webpage')
            await self.sendMessage(response)
            

        elif self.rol == 'manager':
            
            # Get an identifier                    
            self.id = serverManager.newManager(self)

            payload = {
                'response': 'success',
                'id': self.id
            }
            response = {
                'type': 'basic',
                'header': 'handshake',
                'status': 'response',
                'payload': payload
            }
            
            await self.addToGroup(group_name='broadcast')
            await self.sendMessage(response)
    
    async def _getId(self):
        """
        Send the client's identifier to the client
        """
        payload = {
            'id': self.id
        }        
        response = {
            'type': 'basic',
            'header': 'getId',
            'status': 'response',
            'payload': payload
        }
        
        await self.sendMessage(response)
        
    async def _ping(self):
        """
        Send the ping response to the client
        """
        print(f"Ping received from {self.id}")

        response = {
            'type': 'basic',
            'header': 'ping',
            'status': 'response',
            'payload': 'pong'
        }

        await self.sendMessage(response)
        
    async def _getClientsList(self):
        """
        Send the clients list to the client
        """
        payload = {
            'clientList': serverManager.getClientList()
        }
        response = {
            'type': 'basic',
            'header': 'getClientsList',
            'status': 'response',
            'payload': payload
        }

        await self.sendMessage(response)
    #endregion
    
    #region Request communication
    async def _getUAVList(self):
        """
        Send the UAV list to the client
        """
        
        print("SEND UAV List")
        
        payload = {
            'UAVList': serverManager.UAV_list
        }
        response = {
            'type': 'request',
            'header': 'getUAVList',
            'status': 'response',
            'payload': payload
        }

        await self.sendMessage(response)
        
    async def _getMissionList(self):
        """
        Send the UAV list to the client
        """
        payload = {
            'MissionList': serverManager.missions_dict
        }
        response = {
            'type': 'request',
            'header': 'getMissionList',
            'status': 'response',
            'payload': payload
        }

        await self.sendMessage(response)
    #endregion
