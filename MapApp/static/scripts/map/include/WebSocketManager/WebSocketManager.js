
 /**
  * WebSocket class that establish the connection and manage incoming messages
  */
 class WebSocketManager
 {
    constructor(host)
    {
    this.WebSocketConnection = new WebSocketConnection(host);
    this.webSocket = this.WebSocketConnection.webSocket;
    //this.websocketinterface = new WebSocketInterface();

    this.rol = "webpage";
    this.id = null;
    }
 
     /**
      * This function is called when the WebSocket connection is closed.
      * @param {object} webSocket - The WebSocket object.
      * @param {str} error - The error message.
      */
    onError(webSocket, error)
    {
        console.log("Error");
        console.log(error);
    }

     /**
      * This function is called when the WebSocket connection is closed.
      * @param {object} webSocket - The WebSocket object.
      * @param {str} close_status_code - The close status code.
      * @param {str} close_msg - The close message.
      */
    onClose(webSocket, close_status_code, close_msg)
    {
        console.log("Close");
        console.log(`Status code: ${close_status_code}`);
        console.log(`Message: ${close_msg}`);
    }
 
     /**
      * This function is called when the WebSocket connection is opened and start the handshake process.
      * @param {object} webSocket - The WebSocket object.
      */
    onOpen(webSocket)
    {
        this.handshake();
    }
 
     /**
      * This function is called when a message is received from the WebSocket and handle it.
      * @param {object} webSocket - The WebSocket object.
      */
    onMessage(msg)
    {
        let payload = null;

        // console.log("Message recived");
        // console.log(msg);

        switch (msg['type']) {
 
            case 'basic':
                payload = msg['payload'];
                switch (msg['header']) {
                    case 'handshake':
                        if (payload['response'] == 'success') {
                            console.log("Handshake");
                            this.id = payload['id'];
                            this.getUAVList();
                            this.getMissions();
                        } else {
                            console.log("Handshake failed");
                        }
                        break;
                    case 'ping':
                        console.log(`Get:`);
                        console.log(payload);
                        break;
                    case 'getId':
                        console.log(`Get id:`);
                        this.id = payload['id'];
                        console.log(payload);
                        break;
                    case 'getClientsList':
                        console.log(`Get client list:`);
                        console.log(payload);
                        break;
                    default:
                        console.log("Unknown basic message");
                        console.log(msg);
                        break;
                }
                break;
            
            case 'info':
                payload = msg['payload'];
                switch (msg['header']) {
                    case 'newUAVList':
                        //this.websocketinterface.newUAVList(payload);
                        break;

                    case 'newUAV':
                        //this.websocketinterface.newUAV(payload);
                        break;

                    case 'UAVPose':
                        //this.websocketinterface.UAVPose(payload);
                        break;

                    case 'UAVOdom':
                        //this.websocketinterface.UAVOdom(payload);
                        break;

                    case 'UAVDesiredPath':
                        //this.websocketinterface.UAVDesiredPath(payload);
                        break;

                    case 'UAVSensors':
                        //this.websocketinterface.UAVSensors(payload);
                        break;

                    default:
                        console.log("Unknown info message");
                        console.log(msg);
                        break;
                }
                break;
            
            case 'request':
                payload = msg['payload'];
                switch (msg['header']) {
                    case 'getUAVList':
                        //this.websocketinterface.newUAVList(payload);
                        break;
                    case 'getMissionList':
                        //this.websocketinterface.newMissionList(payload);
                        break;
                    case 'confirmMission':
                        //this.websocketinterface.newMission(payload);
                        break;
                    default:
                        console.log("Unknown request message");
                        console.log(msg);
                        break;
                }
                break;

            default:
                console.log("Unknown message");
                console.log(msg);
                break;
            }
    }
 
     /**
      * Send a message to the WebSocket server.
      * @param {dict} msg - JSON-decoded message to be sent.
      * @param {int} to - The id of the client to send the message to.
      */
    send(msg, to=null)
    {
        // If the client is logged in, add its id to the message and the destination
        if (this.id != null) {
            msg['id'] = this.id;

            if (to != null) {
                msg['to'] = to;
            } else {
                msg['to'] = 0; // 0 means server id
            }
        }

        this.webSocket.send(JSON.stringify({'message': msg}));
    }

    sendBasic(header, payload, to=null)
    {
        this.send({
            'type': 'basic',
            'header': header,
            'status': 'request',
            'payload': payload
        }, to);
    }

     /**
      * Send a handshake message to the WebSocket server.
      */
    handshake()
    {
        console.log("Handshake");
        this.sendBasic('handshake', {'rol': 'webpage'})
    }
 
     /**
      * Send a request of id message to the WebSocket server.
      */
    getId()
    {
        this.sendBasic('getId', {});
    }
 
     /**
      * Send a ping message to the WebSocket server.
      */
    ping()
    {
        this.sendBasic('ping', {});
    }

    sendRequest(header, payload, to=null)
    {
        this.send({
            'type': 'request',
            'header': header,
            'status': 'request',
            'payload': payload
        }, to);
    }
 
     /**
      * Send a request of clients list message to the WebSocket server.
      */
    getClientList()
    {
        this.sendRequest('getClientsList', {});
    }
 
     /**
      * Send a request of UAV list to the WebSocket server.
      */
    getUAVList()
    {
        this.sendRequest('getUAVList', {});
    }
     
    getMissions()
    {
        this.sendRequest('getMissionList', {});
    }
}


/**
 * WebSocket manager that establish the connection and manage the communication
 */
class WebSocketConnection 
{
    constructor(host)
    {
        // Listen for messages
        this.webSocket           = new WebSocket(host);
        this.webSocket.onopen    = this.onOpen;
        this.webSocket.onclose   = this.onClose;
        this.webSocket.onerror   = this.onError;
        this.webSocket.onmessage = this.onMessage;
    }
  
    /**
     * This function is called when the WebSocket connection is closed.
     * @param {object} webSocket - The WebSocket object.
     * @param {str} error - The error message.
     */
    onError(webSocket, error)
    {
        WEB_SOCKET_MANAGER.onError(webSocket, error);
    }

    /**
     * This function is called when the WebSocket connection is closed.
     * @param {object} webSocket - The WebSocket object.
     * @param {str} close_status_code - The close status code.
     * @param {str} close_msg - The close message.
     */
    onClose(webSocket, close_status_code, close_msg)
    {
        WEB_SOCKET_MANAGER.onClose(webSocket, close_status_code, close_msg);
    }

    /**
     * This function is called when the WebSocket connection is opened.
     * @param {object} webSocket - The WebSocket object.
     */
    onOpen(webSocket)
    {
        WEB_SOCKET_MANAGER.onOpen(webSocket);
    }

    /**
     * This function is called when a message is received from the WebSocket.
     * @param {object} webSocket - The WebSocket object.
     */
    onMessage(webSocket)
    {
        WEB_SOCKET_MANAGER.onMessage(JSON.parse(webSocket.data).message);
    }
}
