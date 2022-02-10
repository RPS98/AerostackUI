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

        this.UAV_LIST = new UAVList();
        this.MISSION_LIST = new MissionList();
        this.MISSION_DRAW_LIST = new MissionList();
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

    callUavListCallbacks() {
        for (let i = 0; i < this.uavListCallbacks.length; i++) {
            this.uavListCallbacks[i]();
        }
    }

    callUavCallbacks(id) {
        for (let i = 0; i < this.uavCallbacks.length; i++) {
            this.uavCallbacks[i](id);
        }
    }
    
    getUavDict(id)
    {
        return this.UAV_LIST.getUavDict(id);
    }

    getAllUavList() {
        return this.UAV_LIST.getAllUavList();
    }

    getAllUavDict() {
        return this.UAV_LIST.getAllUavDict();
    }

    removeUav(id) {
        this.UAV_LIST.removeUav(id);
        this.callUavListCallbacks();
    }

    addUav(id, state, pose, odom, desiredPath, sensors) {
        if (id in this.getAllUavList()) {
            this.UAV_LIST.setUav(id, state, pose, odom, desiredPath, sensors);
            this.callUavCallbacks(id);
        } else {
            this.UAV_LIST.addUav(id, state, pose, odom, desiredPath, sensors);
            this.callUavListCallbacks();
        }
    }

    _updateUav(id, method, value) {
        if (id in this.getAllUavList()) {
            method(id, value);
            this.callUavCallbacks(id);
        }
    }

    setUavState(id, state) {
        this._updateUav(id, this.UAV_LIST.setUavState, state);
    }

    setUavPose(id, pose) {
        this._updateUav(id, this.UAV_LIST.setUavPose, pose);
    }

    setUavOdom(id, odom) {
        this._updateUav(id, this.UAV_LIST.setUavOdom, odom);
    }

    setUavDesiredPath(id, desiredPath) {
        this._updateUav(id, this.UAV_LIST.setUavDesiredPath, desiredPath);
    }

    setUavSensors(id, sensors) {
        this._updateUav(id, this.UAV_LIST.setUavSensors, sensors);
    }
    // #endregion

    // #region Mission List
    addMissionCallback(callback) {
        this.missionCallbacks.push(callback);
    }

    addMissionListCallback(callback) {
        this.missionListCallbacks.push(callback);
    }

    callMissionListCallbacks() {
        for (let i = 0; i < this.missionListCallbacks.length; i++) {
            this.missionListCallbacks[i]();
        }
    }

    callMissionCallbacks(id) {
        for (let i = 0; i < this.missionCallbacks.length; i++) {
            this.missionCallbacks[i](id);
        }
    }

    getMissionDict(id) {
        return this.MISSION_LIST.getMissionDict(id);
    }

    getAllMissionList() {
        return this.MISSION_LIST.getAllMissionList();
    }

    getAllMissionDict() {
        return this.MISSION_LIST.getAllMissionDict();
    }

    removeMission(id) {
        this.MISSION_LIST.removeMission(id);
        this.callMissionListCallbacks();
    }

}
