/**
 * UAV and Mission Manager prototype, that manage income information from server and call desired callbacks.
 */
class ManagerPrototype {

    /**
     * Create a new instance of the class ManagerPrototype.
     * @param {string} infoAdd - Name of the header of the info message that will be received from server when a parameter is add/modified.
     * @param {string} infoSet - Name of the header of the info message that will be received from server when the information is set/reset.
     * @param {string} infoGet - Name of the header of the request message that will be received from server when the information is requested.
     **/
    constructor(infoAdd, infoSet, infoGet) {

        /**
         * List of colors for each id of the information.
         */
        this.colors = [
            ['#DAE8FC', '#6C8EBF'], // blue
            ['#D5E8D4', '#82B366'], // green
            ['#FFE6CC', '#D79B00'], // orange
            ['#FFF2CC', '#FFF2CC'], // yellow
            ['#F8CECC', '#F8CECC'], // red
            ['#E1D5E7', '#9673A6'], // violet
        ];

        /**
         * Smart list with the information recived from server.
         * @type {SmartList}
         * @private
         */
        this.LIST = new SmartList();

        /**
         * List of callbacks when a parameter is modified
         * @type {array}
         * @private
         */
        this._infoChangeCallbacks = [];

        /**
         * List of callbacks when a object is added or removed
         * @type {array}
         * @private
         */
        this._infoAddCallbacks = [];

        /**
         * List of callbacks when the a parameter is changed
         * @type {array}
         * @private
         */
        this._paramChangeCallbacks = [];

        // Callbacks for income information from server
        M.WS.addCallback('info', infoAdd, this._onInfo.bind(this), 'infoAdd');
        M.WS.addCallback('info', infoSet, this._onInfo.bind(this), 'infoSet');
        M.WS.addCallback('request', infoGet, this._onInfoGet.bind(this));
    }

    // #region Public methods

    /**
     * Return a list of two colors for each id of the information.
     * @param {any} id - Id of the information.
     * @returns {list} - List of two colors, one for the background and one for the border.
     */
    getColors(id) {
        return this.colors[this.getInfoList().indexOf(id) % this.colors.length];
    }

    /**
     * Add a function callback when any parameter is changed.
     * @param {function} callback - Function to be called when the parameter is changed.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addInfoChangeCallback(callback, ...args) {
        this._infoChangeCallbacks.push([callback, args]);
    }

    /**
     * Add a function callback when new information is added.
     * @param {function} callback - Function to be called when the information is added.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addInfoAddCallback(callback, ...args) {
        this._infoAddCallbacks.push([callback, args]);
    }

    /**
     * Add a function callback when the desired parameter is changed.
     * @param {string} param - Parameter to be watched.
     * @param {function} callback - Function to be called when the parameter is changed.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addInfoParamCallback(param, callback, ...args) {
        this._paramChangeCallbacks.push([param, callback, args]);
    }

    /**
     * Get the list of id of the information.
     * @return {array} - List of id of the information.
     */
    getInfoList() {
        return this.LIST.getList();
    }

    /**
     * Get the dictionary with the information for each id.
     * @return {dict} - Dictionary with the information for each id.
     * @access public
     */
    getInfoDict() {
        return this.LIST.getDict();
    }

    /**
     * Return the information in a dictionary of the given id.
     * @param {any} id - Id of the information.
     * @return {dict} - Dictionary with the information. If the id is not found, return null.
     * @access public
     */
    getInfoDictById(id) {
        return this.LIST.getDictById(id);
    }

    /**
     * Remove the information with the given id.
     * @param {any} id - Id of the information.
     * @return {void}
     * @access public 
     */
    removeInfoById(id) {
        this.LIST.removeObject(id);
        this._callCallbacks(this._infoAddCallbacks, id);
    }

    // #endregion

    // #region Private methods

    /**
     * Callback to info message with header infoAdd, that add/updates the information, or
     * with header infoSet, that set the information, and calls the callbacks.
     * @param {dict} payload - payload of the info message
     * @return {void} 
     * @access private
     */
    _onInfo(payload, type) {

        if (!this.LIST.getList().includes(payload['id'])) {
            this.LIST.addObject(payload['id'], payload);
            Utils.callCallbacks(this._infoAddCallbacks, payload['id']);
        } else {
            if (type[0] == 'infoAdd') {
                this.LIST.updateObject(payload['id'], payload);
            } else if (type[0] == 'infoSet') {
                this.LIST.addObject(payload['id'], payload);
            }
            Utils.callCallbacks(this._infoChangeCallbacks, payload['id']);
        }

        for (let key in payload) {
            Utils.callCallbackByParam(this._paramChangeCallbacks, key, payload[key], payload['id']);
        }
    }

    /**
     * Callback to request message with header infoGet, that get the list of information and calls the callbacks.
     * @param {dict} payload - payload of the request message
     * @return {void} 
     * @access private
     */
    _onInfoGet(payload) {
        for (let key in payload['list']) {
            this._onInfo(payload['list'][key], 'infoSet');
        }
    }
    // #endregion
}


/**
 * Mission Manager that extends the ManagerPrototype to manage confirm and reject mission messages.
 */
class MissionManager extends ManagerPrototype {

    /**
     * Create a new instance of the class MissionManager.
     * @param {string} missionConfirm - Name of the header of the info message that will be received from server when a mission is confirmed/rejected.
     * @param {string} missionAdd - Name of the header of the info message that will be received from server when a parameter of the mission is add/modified.
     * @param {string} missionSet - Name of the header of the info message that will be received from server when a mission is set/reset.
     * @param {string} missionGet - Name of the header of the request message that will be received from server when the mission list is requested.
     */
    constructor(missionConfirm, missionAdd, missionSet, missionGet) {
        super(missionAdd, missionSet, missionGet);

        /**
         * List of callbacks when a mission is confirmed/rejected.
         * @type {array}
         * @private
         */
        this._missionConfirmCallbacks = [];

        // Callback for confirm mission information from server
        M.WS.addCallback('request', missionConfirm, this._onMissionConfirm.bind(this));
    }

    // #region Public methods

    /**
     * Add a function callback when a mission is confirmed by the server.
     * @param {function} callback - Function to be called when the mission is confirmed/rejected.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addMissionConfirmCallback(callback, ...args) {
        this._missionConfirmCallbacks.push([callback, args]);
    }

    // #endregion

    // #region Private methods

    /**
     * Callback to request message with header missionConfirm, that get the response of the mission confirmation.
     * @param {dict} payload - payload of the request message
     * @return {void} 
     * @access private
     */
    _onMissionConfirm(payload) {
        if (payload['status'] == 'confirmed') {
            Utils.callCallbacks(this._missionConfirmCallbacks, payload);
        } else if (payload['status'] == 'rejected') {
            // TODO: manage reject
            console.log('Mission rejected');
        }
    }

    // #endregion
}


/**
 * Map manager that create the map, start the communication with the server and manage sidebars.
 */
class MapManager {

    /**
     * Create the instance of the MapManager.
     * @param {list} mapCenter - List of the coordinates [latitude, longitude] of the center of the map.
     * @param {number} zoom - Initial zoom of the map.
     * @param {string} host - Host of the server.
     */
    constructor(mapCenter, zoom, host) {

        /**
         * Smart list with the information recived from server.
         * @type {L.Map} - Map of the library Leaflet (Reference: https://leafletjs.com/).
         * @public
         */
        this.MAP = new L.Map('mapid').setView(mapCenter, zoom);

        /**
         * Layer control of the map.
         * @type {L.control} - Control of the library Leaflet (Reference: https://leafletjs.com/).
         * @private
         */
        this._layer_control = L.control.layers({
            "hybrid": L.tileLayer('http://{s}.google.com/vt/lyrs=s,h&x={x}&y={y}&z={z}', {
                maxZoom: 22,
                subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
            }).addTo(this.MAP),
            "streets": L.tileLayer('http://{s}.google.com/vt/lyrs=m&x={x}&y={y}&z={z}', {
                maxZoom: 22,
                subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
            }),
            "satellite": L.tileLayer('http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
                maxZoom: 22,
                subdomains: ['mt0', 'mt1', 'mt2', 'mt3'],
            }),
            "terrain": L.tileLayer('http://{s}.google.com/vt/lyrs=p&x={x}&y={y}&z={z}', {
                maxZoom: 22,
                subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
            })

        }, {}, { position: 'topleft', collapsed: false }).addTo(this.MAP);

        // Create sidebars HTML elements
        this._initializeSideBars();

        // Initialize connection to server
        /**
         * Web Socket Manager instance to manage the connection with the server.
         * @type {WebSocketManager}
         * @public
         */
        this.WS = new WebSocketManager(host);

        // Add callbacks to the WebSocketManager when a basic message of handshake is received
        this.WS.addCallback('basic', 'handshake', this._onHandshake.bind(this));

        // Layers created manager
        /**
         * List of callbacks when a layer is created.
         * @type {array}
         * @private
         */
        this._mapPmCreateCallbacks = [];

        this.MAP.on('pm:create', function (e) {

            // Change layer color if the option color is set
            if ('color' in e.layer.pm.options) {
                e.layer.setStyle({ color: e.layer.pm.options['color'] });
                if ('color' in e.layer.options) {
                    e.layer.options['color'] = e.layer.options['color'];
                }
            }

            Utils.callCallbacks(M._mapPmCreateCallbacks, e);
        });
    }

    // #region Public methods

    /**
     * Initialize UAV Manager and Mission Manager.
     * @return {void}
     * @access public
     */
    initialize() {
        this.UAV_MANAGER = new ManagerPrototype('uavInfo', 'uavInfoSet', 'getUavList');
        this.MISSION_MANAGER = new MissionManager('missionConfirm', 'missionInfo', 'missionInfoSet', 'getMissionList');
    }

    /**
     * Get all layers of the map and return them in a list.
     * @returns {array} - List of layers.
     * @access public
     */
    getLayers() {
        return L.PM.Utils.findLayers(this.MAP);
    }

    /**
     * Add a function callback when a layer is created.
     * @param {function} callback - Function to be called when the layer is created.
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addPmCreateCallback(callback, ...args) {
        this._mapPmCreateCallbacks.push([callback, args]);
    }

    // #endregion

    // #region Private methods

    // #region Web Socket Callbacks

    /**
     * Callback to basic message with header handshake, and ask for the mission and UAVs state.
     * @param {dict} payload - payload of the handshake message
     * @return {void}
     * @access private
     */
    _onHandshake(payload) {
        if (payload['response'] == 'success') {
            this.WS.sendRequestGetUavList();
            this.WS.sendRequestGetMissionList();
        }
    }

    // #endregion

    // #region Side Bars

    _initializeSideBars() {
        this._initializeLefSideBar();
        this._initializeRightSideBar();
    }

    _initializeLefSideBar() {
        this.sidebar_left = L.control.sidebar({
            autopan: true,             // whether to maintain the centered map point when opening the sidebar
            closeButton: true,         // whether t add a close button to the panes
            container: 'sideBar-left', // the DOM container or #ID of a predefined sidebar container that should be used
            position: 'left',
        }).addTo(this.MAP);
    }

    _initializeRightSideBar() {
        this.sidebar_right = L.control.sidebar({
            autopan: false,
            closeButton: true,
            container: 'sideBar-right',
            position: 'right',
        }).addTo(this.MAP);
    }

    // #endregion

    // #endregion
}
