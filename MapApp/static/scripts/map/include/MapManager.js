/**
 * UAV and Mission Manager prototype, that manage income information from server and call desired callbacks.
 */
class ManagerPrototype extends SmartListCallbacks {

    /**
     * Create a new instance of the class ManagerPrototype.
     * @param {string} infoAdd - Name of the header of the info message that will be received from server when a parameter is add/modified.
     * @param {string} infoSet - Name of the header of the info message that will be received from server when the information is set/reset.
     * @param {string} infoGet - Name of the header of the request message that will be received from server when the information is requested.
     **/
    constructor(infoAdd, infoSet, infoGet) {

        super();

        /**
         * List of colors for each id of the information.
         */
        this.colors = [
            ['#FFE6CC', '#D79B00'], // orange
            ['#F8CECC', '#F8CECC'], // red
            ['#D5E8D4', '#82B366'], // green
            ['#DAE8FC', '#6C8EBF'], // blue
            ['#FFF2CC', '#FFF2CC'], // yellow
            ['#E1D5E7', '#9673A6'], // violet
        ];

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
        return this.colors[super.getList().indexOf(id) % this.colors.length];
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

        if (!super.getList().includes(payload['id'])) {
            super.addObject(payload['id'], payload);
        } else {
            if (type[0] == 'infoAdd') {
                super.updateObject(payload['id'], payload, false);
            } else if (type[0] == 'infoSet') {
                super.updateObject(payload['id'], payload, true);
            }
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

        // Add a layer control to the map, with the latitude and longitude of the mouse
        L.control.coordinates({
            position: "bottomright",
            decimals: 5,
            decimalSeperator: ".",
            labelTemplateLat: "Lat: {y}",
            labelTemplateLng: "Lng: {x}",
            useLatLngOrder: true
        }).addTo(this.MAP);

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
        this.addMapCallback('pm:create', this._pmOnCreateCallback);

        this._uavPickerCallbackList = [];
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

        this.UAV_MANAGER.addInfoAddCallback(this._updateUavPickerListCallback.bind(this));
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
     * Add a function callback to map listener
     * @param {function} callback - Function to be called when the map event happend
     * @param  {...any} args - Arguments to be passed to the callback.
     * @return {void} 
     * @access public
     */
    addMapCallback(type, callback, ...args) {
        this.MAP.on(type, function (e) {
            callback(args, e);
        });
    }

    getUavPickerDict(type, id, othersElements=[], callback, ...args) {
        let uavList = M.UAV_MANAGER.getList();

        if (uavList.length > 0) {
            for (let i = 0; i < othersElements.length; i++) {
                if (!othersElements[i][1]) {
                    othersElements.splice(i, 1);
                }
            }
        }

        this._uavPickerCallbackList.push([id, callback, othersElements, args]);
        
        if (othersElements.length > 0) {
            let list = [];
            for (let i = 0; i < othersElements.length; i++) {
                list.push(othersElements[i][0]);
            }
            return HTMLUtils.addDict('checkBoxes', `${id}`, { 'class': 'UavPicker' }, type, list.concat(uavList));
        }
        return HTMLUtils.addDict('checkBoxes', `${id}`, { 'class': 'UavPicker' }, type, uavList);
    }

    uavPickerInitiliazeCallback(id) {
        // Add callback to the checkbox
        for (let i = 0; i < this._uavPickerCallbackList.length; i++) {
            let divId = this._uavPickerCallbackList[i][0];
            let callback = this._uavPickerCallbackList[i][1];
            let othersElements = this._uavPickerCallbackList[i][2];
            let userargs = this._uavPickerCallbackList[i][3];

            if (divId == id) {
                for (let j = 0; j < othersElements.length; j++) {
                    let name = othersElements[j][0];
                    let checkBoxId = id + '-'+ name + '-Input';
                    this. _uavPickerAddCallback(checkBoxId, callback, userargs);
                }
                let uavList =  M.UAV_MANAGER.getList();
                for (let j = 0; j < uavList.length; j++) {
                    let uavName = uavList[j];
                    let uavCheckBoxId = id + '-'+ uavName + '-Input';
                    this. _uavPickerAddCallback(uavCheckBoxId, callback, userargs);
                }
            }
        }
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

    _pmOnCreateCallback(args, e) {
        let layers = M.getLayers();

        // Change layer color if the option color is set
        if ('color' in e.layer.pm.options) {
            e.layer.setStyle({ color: e.layer.pm.options['color'] });
            if ('color' in e.layer.options) {
                e.layer.options['color'] = e.layer.options['color'];
            }
        }
    }

    _uavPickerAddCallback(checkBoxId, callback, userargs) {
        document.getElementById(checkBoxId).addEventListener('change', function () {
            let inputId = this.id.split('-');
            let uavName = inputId[inputId.length - 1];
            let value = this.checked;

            callback(uavName, value, userargs);
        });
    }

    _updateUavPickerList(myargs, args, picker) {

        // Update the checkbox HTML
        let childId = picker.children[0].id;
        let type = document.getElementById(childId + '-Input').type;

        HTMLUtils.addCheckBox(picker.id, picker.id, type, args[0]);

        // Add callback to the checkbox
        for (let i = 0; i < this._uavPickerCallbackList.length; i++) {
            let divId = this._uavPickerCallbackList[i][0];
            let callback = this._uavPickerCallbackList[i][1];
            let othersElements = this._uavPickerCallbackList[i][2];
            let userargs = this._uavPickerCallbackList[i][3];

            if (divId == picker.id) {
                let checkBoxId = picker.id + '-'+ args[0] + '-Input';
                
                this. _uavPickerAddCallback(checkBoxId, callback, userargs);

                // Remove defaults checkboxes
                for (let i = 0; i < othersElements.length; i++) {
                    let name = othersElements[i][0];
                    let keep = othersElements[i][1];
                    if (!keep) {
                        let checkBoxId = picker.id + '-' + name;
                        document.getElementById(checkBoxId).remove();
                        this._uavPickerCallbackList[i][2].splice(i, 1);
                    }
                }
            }
        }
    }

    _updateUavPickerListCallback(myargs, args) {
        let pickers = document.getElementsByClassName('UavPicker');
        for (let i = 0; i < pickers.length; i++) {
            this._updateUavPickerList(myargs, args, pickers[i]);
        }
    }

    // #endregion
}
