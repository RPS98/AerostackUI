/**
 * UAV and Mission Manager prototype, that manage income information from server and call desired callbacks.
 */
class ManagerPrototype {
    /**
     * Create a new instance of the ManagerPrototype.
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
         * @type {Array}
         * @private
         */
        this._infoChangeCallbacks = [];

        /**
         * List of callbacks when a object is added or removed
         * @type {Array}
         * @private
         */
        this._infoAddCallbacks = [];

        /**
         * List of callbacks when the a parameter is changed
         * @type {Array}
         * @private
         */
        this._paramChangeCallbacks = [];

        // Callbacks for income information from server
        M.WS.addCallback('info',    infoAdd, this._onInfo.bind(this), 'infoAdd');
        M.WS.addCallback('info',    infoSet, this._onInfo.bind(this), 'infoSet');
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
     * @return {Array} - List of id of the information.
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
     * Constructor of the MissionManager class.
     * @param {string} missionConfirm - Name of the header of the info message that will be received from server when a mission is confirmed/rejected.
     * @param {string} missionAdd - Name of the header of the info message that will be received from server when a parameter of the mission is add/modified.
     * @param {string} missionSet - Name of the header of the info message that will be received from server when a mission is set/reset.
     * @param {string} missionGet - Name of the header of the request message that will be received from server when the mission list is requested.
     */
    constructor(missionConfirm, missionAdd, missionSet, missionGet) {
        super(missionAdd, missionSet, missionGet);
        M.WS.addCallback('request', missionConfirm, this._onMissionConfirm.bind(this));
        this._missionConfirmCallbacks = [];
    }

    // #region Public methods
    addMissionConfirmCallback(callback, ...args) {
        this._missionConfirmCallbacks.push([callback, args]);
    }

    // #endregion
    // #region Private methods
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


class MissionDrawManager
{
    constructor() {
        this.MISSION_DRAW_LIST = new SmartList();
        this.missionDrawCallbacks = []; // When a mission drawn is modified
        this.missionDrawListCallbacks = []; // When a mission drawn is added or removed
    }

    // #region Mission Draw List
    addMissionDrawCallback(callback, ...args) {
        this.missionDrawCallbacks.push([callback, args]);
    }

    addMissionDrawListCallback(callback, ...args) {
        this.missionDrawListCallbacks.push([callback, args]);
    }

    getMissionDrawDicById(id)
    {
        if (id in this.getMissionDrawList()) {
            return this.MISSION_DRAW_LIST.getDictById(id);
        } else {
            return null;
        }
    }

    getMissionDrawList() {
        return this.MISSION_DRAW_LIST.getList();
    }

    getMissionDrawDict() {
        return this.MISSION_DRAW_LIST.getDict();
    }

    removeMissionDraw(id) {
        this.MISSION_DRAW_LIST.removeObject(id);
        this._callCallbacks(this.missionDrawListCallbacks);
    }

    addMissionDraw(id, state, uavList, layers) {
        if (id in this.getMissionDrawList()) {
            this.getMissionDrawDicById(id).setMissionDraw(id, state, uavList, layers);
            this._callCallbacks(this.missionDrawCallbacks, id);
        } else {
            this.MISSION_DRAW_LIST.addObject(id, new Mission(id, state, uavList, layers) );
            this._callCallbacks(this.missionDrawListCallbacks);
        }
    }

    setMissionDrawState(id, state) {
        this.getMissionDrawDicById(id).setMissionDrawState(state);
        this._callCallbacks(this.missionDrawCallbacks, id);
    }

    setMissionDrawUavList(id, uavList) {
        this.getMissionDrawDicById(id).setMissionDrawUavList(uavList);
        this._callCallbacks(this.missionDrawCallbacks, id);
    }

    setMissionDrawLayers(id, layers) {
        this.getMissionDrawDicById(id).setMissionDrawLayers(layers);
        this._callCallbacks(this.missionDrawCallbacks, id);
    }
    // #endregion
}

class MapManager
{
    constructor(map_center, zoom, host)
    {
        this.MAP = new L.Map('mapid').setView(map_center, zoom);

        this.uavTilesAll = {'All': L.featureGroup().addTo(this.MAP)};
        
        // add tile layers
        this.layer_control = L.control.layers({
            "hybrid": L.tileLayer('http://{s}.google.com/vt/lyrs=s,h&x={x}&y={y}&z={z}',{
                maxZoom: 22,
                subdomains:['mt0','mt1','mt2','mt3']
            }).addTo(this.MAP),
            "streets": L.tileLayer('http://{s}.google.com/vt/lyrs=m&x={x}&y={y}&z={z}',{
                maxZoom: 22,
                subdomains:['mt0','mt1','mt2','mt3']
            }),
            "satellite": L.tileLayer('http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',{
                maxZoom: 22,
                subdomains:['mt0','mt1','mt2','mt3'],
            }),
            "terrain": L.tileLayer('http://{s}.google.com/vt/lyrs=p&x={x}&y={y}&z={z}',{
                maxZoom: 22,
                subdomains:['mt0','mt1','mt2','mt3']
            })
            
        }, this.uavTilesAll, { position: 'topleft', collapsed: false }).addTo(this.MAP);

        // {% include 'include/SideBar/SideBar.html' %}

        this.initializeSideBars();
        
        // Initialize connection to server
        this.WS = new WebSocketManager(host);
        this.WS.addCallback('basic', 'handshake', this.onHandshake.bind(this));

        this.colors = [
            '#DAE8FC', // blue
            '#D5E8D4', // green
            '#FFE6CC', // orange
            '#FFF2CC', // yellow
            '#F8CECC', // red
            '#E1D5E7', // violet
        ];

        // Color grey
        this.drawColor = '#B3B3B3';

        this.mapPmCreateCallbacks = [];

        this.MAP.on('pm:create', function (e) {

            if ('color' in e.layer.pm.options) {
                e.layer.setStyle({color: e.layer.pm.options['color']});
                if ('color' in e.layer.options) {
                    e.layer.options['color'] = e.layer.options['color'];
                }
            }

            Utils.callCallbacks(M.mapPmCreateCallbacks, e);
        });

        
    }

    initialize() {
        this.UAV_MANAGER = new ManagerPrototype('uavInfo', 'uavInfoSet', 'getUavList');
        this.MISSION_MANAGER = new MissionManager('missionConfirm', 'missionInfo', 'missionInfoSet', 'getMissionList');
        this.MISSIOn_DRAW_MANAGER = new MissionDrawManager();
    }


    addPmCreateCallback(callback, ...args) {
        this.mapPmCreateCallbacks.push([callback, args]);
    }

    // #region WebScoket Callbacks
    onHandshake(payload) {
        if (payload['response'] == 'success') {
            this.WS.requestGetUavList();
            this.WS.requestGetMissionList();
        }
    }

    // #endregion
    
    // #region Side Bars
    initializeSideBars() {
        this.initializeLefSideBar();
        this.initializeRightSideBar();
    }

    initializeLefSideBar() {
        this.sidebar_left = L.control.sidebar({
            autopan: true,             // whether to maintain the centered map point when opening the sidebar
            closeButton: true,         // whether t add a close button to the panes
            container: 'sideBar-left', // the DOM container or #ID of a predefined sidebar container that should be used
            position: 'left',
        }).addTo(this.MAP);
    }

    initializeRightSideBar() {
        this.sidebar_right = L.control.sidebar({
            autopan: false,
            closeButton: true,
            container: 'sideBar-right',
            position: 'right',
        }).addTo(this.MAP);
    }

    getLayers() {
        return L.PM.Utils.findLayers(this.MAP);
    }
    // #endregion
}
