class UAVManager
{
    constructor() {
        this.UAV_LIST = new SmartList();
        this.uavparamCallbacks = [];
        this.uavCallbacks = [];     // When a UAV is modified
        this.uavListCallbacks = []; // When a UAV is added or removed
        this.initialize();

        this.colors = [
            ['#DAE8FC', '#6C8EBF'], // blue
            ['#D5E8D4', '#82B366'], // green
            ['#FFE6CC', '#D79B00'], // orange
            ['#FFF2CC', '#FFF2CC'], // yellow
            ['#F8CECC', '#F8CECC'], // red
            ['#E1D5E7', '#9673A6'], // violet
        ];
    }

    initialize() {
        M.WS.addCallback('info', 'uavInfo', this.onUavInfo.bind(this));
        M.WS.addCallback('info', 'uavInfoSet', this.onUavInfoSet.bind(this));

        M.WS.addCallback('request', 'getUavList', this.onUAVListUpdate.bind(this));
    }

    _callCallbacks(callbackList, ...args) {
        for (let i = 0; i < callbackList.length; i++) {
            callbackList[i][0](callbackList[i][1], ...args);
        }
    }

    _callCallbackByParam(id, param, value) {
        for (let i=0; i<this.uavparamCallbacks.length; i++) {
            if (this.uavparamCallbacks[i][0] == param || this.uavparamCallbacks[i][0] == '*') {
                this.uavparamCallbacks[i][1](id, param, value);
            }
        }
    }

    _callCallbacksParam(payload) {
        for (let key in payload) {
            this._callCallbackByParam(payload['id'], key, payload[key]);
        }
    }

    getIconColor(id) {
        return this.colors[this.getUavList().indexOf(id) % this.colors.length];
    }

    getColor(id) {
        return M.colors[this.getUavList().indexOf(id) % M.colors.length][0];
    }

    // #region WebScoket Callbacks
    onUavInfo(payload) {
        if (!this.UAV_LIST.getList().includes(payload['id'])) {
            this.UAV_LIST.addObject(payload['id'], payload);
            this._callCallbacks(this.uavListCallbacks, payload['id']);
        } else {
            this.UAV_LIST.updateObject(payload['id'], payload);
            this._callCallbacks(this.uavCallbacks, payload['id']);
        }
        this._callCallbacksParam(payload);
    }

    onUavInfoSet(payload) {
        if (!this.UAV_LIST.getList().includes(payload['id'])) {
            this.UAV_LIST.addObject(payload['id'], payload);
            this._callCallbacks(this.uavListCallbacks, payload['id']);
        } else {
            this.UAV_LIST.addObject(payload['id'], payload);
            this._callCallbacks(this.uavCallbacks, payload['id']);
        }
        this._callCallbacksParam(payload);
    }

    onUAVListUpdate(payload) {
        for (let key in payload['uavList']) {
            let uav = payload['uavList'][key];
            this.onUavInfo(uav);
        }
    }
    // #endregion

    // #region UAV List
    addUavCallback(callback, ...args) {
        this.uavCallbacks.push([callback, args]);
    }

    addUavListCallback(callback, ...args) {
        this.uavListCallbacks.push([callback, args]);
    }

    addUavParamCallback(param, callback, ...args) {
        this.uavparamCallbacks.push([param, callback, args]);
    }
    
    getUavDictById(id)
    {
        if (this.getUavList().indexOf(id) !== -1) {
            return this.UAV_LIST.getDictById(id);
        } else {
            return null;
        }
    }

    getUavList() {
        return this.UAV_LIST.getList();
    }

    getUavDict() {
        return this.UAV_LIST.getDict();
    }

    removeUav(id) {
        this.UAV_LIST.removeObject(id);
        this._callCallbacks(this.uavListCallbacks, id);
    }

    // #endregion
}

class MissionManager
{

    constructor() {
        this.MISSION_LIST = new SmartList();
        this.missionCallbacks = [];     // When a mission is modified
        this.missionListCallbacks = [];     // When a mission is added or removed
    }

    _callCallbacks(callbackList, ...args) {
        for (let i = 0; i < callbackList.length; i++) {
            callbackList[i][0](callbackList[i][1], ...args);
        }
    }

    // #region Mission List
    addMissionCallback(callback, ...args) {
        this.missionCallbacks.push([callback, args]);
    }

    addMissionListCallback(callback, ...args) {
        this.missionListCallbacks.push([callback, args]);
    }

    getMissionDicById(id)
    {
        if (id in this.getMissionList()) {
            return this.MISSION_LIST.getDictById(id);
        } else {
            return null;
        }
    }

    getMissionList() {
        return this.MISSION_LIST.getList();
    }

    getMissionDict() {
        return this.MISSION_LIST.getDict();
    }

    removeMission(id) {
        this.MISSION_LIST.removeObject(id);
        this._callCallbacks(this.missionListCallbacks);
    }

    addMission(id, state, uavList, layers) {
        if (id in this.getMissionList()) {
            this.getMissionDicById(id).setMission(id, state, uavList, layers);
            this._callCallbacks(this.missionCallbacks, id);
        } else {
            this.MISSION_LIST.addObject(id, new Mission(id, state, uavList, layers) );
            this._callCallbacks(this.missionListCallbacks);
        }
    }

    setMissionState(id, state) {
        this.getMissionDicById(id).setMissionState(state);
        this._callCallbacks(this.missionCallbacks, id);
    }

    setMissionUavList(id, uavList) {
        this.getMissionDicById(id).setMissionUavList(uavList);
        this._callCallbacks(this.missionCallbacks, id);
    }

    setMissionLayers(id, layers) {
        this.getMissionDicById(id).setMissionLayers(layers);
        this._callCallbacks(this.missionCallbacks, id);
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

        // Default color
        // M.MAP.pm.setPathOptions({
        //     color: 'red',
        // });

        this.MAP.on('pm:create', function (e) {            
            if ('color' in e.layer.pm.options) {
                e.layer.setStyle({color: e.layer.pm.options['color']});
                if ('color' in e.layer.options) {
                    e.layer.options['color'] = e.layer.options['color'];
                }
            }
        });
    }

    initialize() {
        this.UAV_MANAGER = new UAVManager();
        this.MISSION_MANAGER = new MissionManager();
        this.MISSIOn_DRAW_MANAGER = new MissionDrawManager();
    }

    _callCallbacks(callbackList, ...args) {
        for (let i = 0; i < callbackList.length; i++) {
            callbackList[i][0](callbackList[i][1], ...args);
        }
    }

    // #region WebScoket Callbacks
    onHandshake(payload) {
        console.log('Handshake received');
        this.WS.requestGetUavList();
        this.WS.requestGetMissionList();
        /*
        this.UAV_MANAGER.addUav('PX 1', 'landed', {'lat': 0, 'lng': 0, 'yaw': 0}, [], [], {});
        this.UAV_MANAGER.addUav('PX 2', 'landed', {'lat': 0, 'lng': 0, 'yaw': 0}, [], [], {});

        this.MISSION_MANAGER.addMission('Mission 1', 'landed', ['PX 1'], []);
        this.MISSION_MANAGER.addMission('Mission 2', 'landed', ['PX 2'], []);
        */
        
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
