/**
 * WebSocket manager that establish the connection and manage the communication
 */
class WebSocketConnection {
    constructor(host) {
        // Listen for messages
        this.webSocket = new WebSocket(host);
        this.webSocket.onopen = this.onOpen;
        this.webSocket.onclose = this.onClose;
        this.webSocket.onerror = this.onError;
        this.webSocket.onmessage = this.onMessage;
    }

    /**
     * This function is called when the WebSocket connection is closed.
     * @param {object} webSocket - The WebSocket object.
     * @param {str} error - The error message.
     */
    onError(webSocket, error) {
        M.WS.onError(webSocket, error);
    }

    /**
     * This function is called when the WebSocket connection is closed.
     * @param {object} webSocket - The WebSocket object.
     * @param {str} close_status_code - The close status code.
     * @param {str} close_msg - The close message.
     */
    onClose(webSocket, close_status_code, close_msg) {
        M.WS.onClose(webSocket, close_status_code, close_msg);
    }

    /**
     * This function is called when the WebSocket connection is opened.
     * @param {object} webSocket - The WebSocket object.
     */
    onOpen(webSocket) {
        M.WS.onOpen(webSocket);
    }

    /**
     * This function is called when a message is received from the WebSocket.
     * @param {object} webSocket - The WebSocket object.
     */
    onMessage(webSocket) {
        M.WS.onMessage(JSON.parse(webSocket.data).message);
    }
}


/**
 * WebSocket class that establish the connection and manage incoming messages
 */
class WebSocketManager {
    constructor(host) {
        this.WebSocketConnection = new WebSocketConnection(host);
        this.webSocket = this.WebSocketConnection.webSocket;
        //this.websocketinterface = new WebSocketInterface();

        this.rol = "webpage";
        this.id = null;

        this.callbacksList = [];
        this.addCallback('basic', 'handshake', this.onHandshake);
        this.addCallback('basic', 'getId', this.onGetId);
        this.addCallback('basic', 'ping', this.onPing);
        this.addCallback('basic', 'getClientsList', this.onGetClientsList);
    }

    /**
     * This function is called when the WebSocket connection is closed.
     * @param {object} webSocket - The WebSocket object.
     * @param {str} error - The error message.
     */
    onError(webSocket, error) {
        console.log("Error");
        console.log(error);
    }

    /**
     * This function is called when the WebSocket connection is closed.
     * @param {object} webSocket - The WebSocket object.
     * @param {str} close_status_code - The close status code.
     * @param {str} close_msg - The close message.
     */
    onClose(webSocket, close_status_code, close_msg) {
        console.log("Close");
        console.log(`Status code: ${close_status_code}`);
        console.log(`Message: ${close_msg}`);
    }

    /**
     * This function is called when the WebSocket connection is opened and start the handshake process.
     * @param {object} webSocket - The WebSocket object.
     */
    onOpen(webSocket) {
        this.handshake();
    }

    /**
     * This function is called when a message is received from the WebSocket and handle it.
     * @param {object} webSocket - The WebSocket object.
     */
    onMessage(msg) {
        console.log("Message received");
        console.log(msg);
        
        let payload = null;
        for (let i = 0; i < this.callbacksList.length; i++) {
            if (this.callbacksList[i]['type'] == msg['type'] && this.callbacksList[i]['header'] == msg['header']) {
                payload = msg['payload'];
                this.callbacksList[i]['callback'](payload);
            }
        }

        // console.log("Message recived");
        // console.log(msg);
        /*
        switch (msg['type']) {


            case 'request':
                payload = msg['payload'];
                switch (msg['header']) {
                    case 'getUAVList':

                        M.addUav('PX 1', 'landed', { 'lat': 0, 'lng': 0, 'yaw': 0 }, [], [], {});

                        //this.websocketinterface.newUAVList(payload);
                        break;
                    case 'getMissionList':

                        M.addMission('Mission 1', 'landed', ['PX 1'], []);
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
        */
    }

    addCallback(type, header, callback) {
        this.callbacksList.push({
            'type': type,
            'header': header,
            'callback': callback
        });
    }

    /**
     * Send a message to the WebSocket server.
     * @param {dict} msg - JSON-decoded message to be sent.
     * @param {int} to - The id of the client to send the message to.
     */
    send(msg, to = null) {
        // If the client is logged in, add its id to the message and the destination
        if (this.id != null) {
            msg['id'] = this.id;

            if (to != null) {
                msg['to'] = to;
            } else {
                msg['to'] = 0; // 0 means server id
            }
        }

        this.webSocket.send(JSON.stringify({ 'message': msg }));
    }

    sendBasic(header, payload, to = null) {
        this.send({
            'type': 'basic',
            'header': header,
            'status': 'request',
            'payload': payload
        }, to);
    }

    sendRequest(header, payload, to = null) {
        this.send({
            'type': 'request',
            'header': header,
            'status': 'request',
            'payload': payload
        }, to);
    }

    // #region Basic messages
    /**
     * Send a handshake message to the WebSocket server.
     */
    handshake() {
        this.sendBasic('handshake', { 'rol': 'webpage' })
    }

    /**
     * Send a request of id message to the WebSocket server.
     */
    getId() {
        this.sendBasic('getId', {});
    }

    /**
     * Send a ping message to the WebSocket server.
     */
    ping() {
        this.sendBasic('ping', {});
    }

    /**
     * Send a request of clients list message to the WebSocket server.
     */
    getClientList() {
        this.sendRequest('getClientsList', {});
    }

    onHandshake(payload) {
        if (payload['response'] == 'success') {
            console.log("Handshake");
            this.id = payload['id'];
        } else {
            throw new Error("Handshake failed");
        }
    }

    onGetId(payload) {
        console.log(`Get id:`);
        this.id = payload['id'];
        console.log(payload);
    }

    onPing(payload) {
        console.log(`Get:`);
        console.log(payload);
    }

    onGetClientsList(payload) {
        console.log(`Get client list:`);
        console.log(payload);
    }

    // #endregion

    // #region Request messages
    requestGetUAVList() {
        this.sendRequest('getUAVList', {});
    }

    requestGetMissionList() {
        this.sendRequest('getMissionList', {});
    }

    requestConfirmMission(missionId) {
        this.sendRequest('confirmMission', { 'missionId': missionId });
    }
}


