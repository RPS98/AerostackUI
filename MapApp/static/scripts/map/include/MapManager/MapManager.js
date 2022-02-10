class MapManager
{
    constructor(map_center, zoom)
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
        
        this.uavCallbacks = [];         // When a UAV is modified
        this.missionCallbacks = [];     // When a mission is modified
        this.missionDrawCallbacks = []; // When a mission drawn is modified

        this.uavListCallbacks = [];         // When a UAV is added or removed
        this.missionListCallbacks = [];     // When a mission is added or removed
        this.missionDrawListCallbacks = []; // When a mission drawn is added or removed

        this.UAV_LIST = new SmartList();
        this.MISSION_LIST = new SmartList();
        this.MISSION_DRAW_LIST = new SmartList();
    }

    _callCallbacks(callbackList, ...args) {
        for (let i = 0; i < callbackList.length; i++) {
            callbackList[i](...args);
        }
    }

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
    // #endregion

    // #region UAV List
    addUavCallback(callback) {
        this.uavCallbacks.push(callback);
    }

    addUavListCallback(callback) {
        this.uavListCallbacks.push(callback);
    }
    
    getUavDicById(id)
    {
        if (id in this.getUavList()) {
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
        this._callCallbacks(this.uavListCallbacks);
    }

    addUav(id, state, pose, odom={}, desiredPath={}, sensors={}) {
        if (id in this.getUavList()) {
            this.getUavDicById(id).setUav(id, state, pose, odom={}, desiredPath={}, sensors={});
            this._callCallbacks(this.uavCallbacks, id);
        } else {
            this.UAV_LIST.addObject(id, new UAV(id, state, pose, odom, desiredPath, sensors));
            this._callCallbacks(this.uavListCallbacks);
        }
    }

    setUavState(id, state) {
        this.getUavDicById(id).setUavState(state);
        this._callCallbacks(this.uavCallbacks, id);
    }

    setUavPose(id, pose) {
        this.getUavDicById(id).setUavPose(pose);
        this._callCallbacks(this.uavCallbacks, id);
    }

    setUavOdom(id, odom) {
        this.getUavDicById(id).setUavOdom(odom);
        this._callCallbacks(this.uavCallbacks, id);
    }

    setUavDesiredPath(id, desiredPath) {
        this.getUavDicById(id).setUavDesiredPath(desiredPath);
        this._callCallbacks(this.uavCallbacks, id);
    }

    setUavSensors(id, sensors) {
        this.getUavDicById(id).setUavSensors(sensors);
        this._callCallbacks(this.uavCallbacks, id);
    }
    // #endregion

    // #region Mission List
    addMissionCallback(callback) {
        this.missionCallbacks.push(callback);
    }

    addMissionListCallback(callback) {
        this.missionListCallbacks.push(callback);
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

    // #region Mission Draw List
    addMissionDrawCallback(callback) {
        this.missionDrawCallbacks.push(callback);
    }

    addMissionDrawListCallback(callback) {
        this.missionDrawListCallbacks.push(callback);
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
