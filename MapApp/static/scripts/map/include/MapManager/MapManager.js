class UAV {
    constructor(id, state, pose, odom, desiredPath, sensors) 
    {
        this.newState(id, state, pose, odom, desiredPath, sensors);
    }

    newState(id, state, pose, odom, desiredPath, sensors) 
    {
        this.landedMarker = null;
        this.odomMarker = null;
        this.poseMarker = null;
        this.desiredPathMarker = null;

        this.UAVId = id;
        this.state = state;
        this.pose = pose;
        this.odom = odom;
        this.desiredPath = desiredPath;
        this.sensors = sensors;

        this.drawUAVPose(this.pose, this.state);
        this.drawUAVOdom(this.odom);
        this.drawDesiredPath(this.desiredPath);
        this.showSensor(this.sensors);
    }

    drawUAVLanded(pose)
    {
        if (this.landedMarker != null) {this.landedMarker.remove();}
        this.landedMarker = UAVDrawing.drawUAVLanded(iApp.map, this.UAVId, pose);
    }
    
    drawUAVOdom(odom)
    {
        if (this.odomMarker != null) {this.odomMarker.remove();}
        this.odom = odom;
        this.odomMarker = UAVDrawing.drawUAVOdom(iApp.map, this.UAVId, this.odom);
    }

    drawUAVPose(pose, state)
    {
        this.pose = pose;
        this.state = state;

        if (this.poseMarker != null) {this.poseMarker.remove();}
        this.poseMarker =UAVDrawing.drawUAVPose(iApp.map, this.UAVId, this.pose);

        if (this.state == 'landed') {
            this.drawUAVLanded(this.pose);
        } 
    }

    drawDesiredPath(desiredPath)
    {
        this.desiredPath = desiredPath;
        if (this.desiredPathMarker != null) {this.desiredPathMarker.remove();}
        this.desiredPathMarker = UAVDrawing.drawDesiredPath(iApp.map, this.UAVId, this.desiredPath);
    }

    showSensor(sensors)
    {
        this.sensors = sensors;
        // console.log("Drawing Sensors");
        // console.log(sensors);
    }
}

class Mission
{
    constructor() {

    }
}

class MapManager
{
    constructor(map_center, zoom)
    {
        map = new L.Map('mapid').setView(map_center, zoom);

        this.uavTilesAll = {'All': L.featureGroup().addTo(map)};
        
        // add tile layers
        this.layer_control = L.control.layers({
            "hybrid": L.tileLayer('http://{s}.google.com/vt/lyrs=s,h&x={x}&y={y}&z={z}',{
                maxZoom: 22,
                subdomains:['mt0','mt1','mt2','mt3']
            }).addTo(map),
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
            
        }, this.uavTilesAll, { position: 'topleft', collapsed: false }).addTo(map);

        // {% include 'include/SideBar/SideBar.html' %}

        this.initializeSideBars();
        
    }

    initializeSideBars() {
        /*
        // Initialize HTML Sidebars 
        let div = document.createElement('div');
        div.setAttribute('id', 'sideBar');
        div.innerHTML = `{% include 'include/SideBar/SideBar.html' %}`

        let article = document.getElementById('main_article');

        article.appendChild(div);
        */
        this.initializeLefSideBar();
        this.initializeRightSideBar();
    }

    initializeLefSideBar() {
        this.sidebar_left = L.control.sidebar({
            autopan: true,             // whether to maintain the centered map point when opening the sidebar
            closeButton: true,         // whether t add a close button to the panes
            container: 'sideBar-left', // the DOM container or #ID of a predefined sidebar container that should be used
            position: 'left',
        }).addTo(map);
    }

    initializeRightSideBar() {
        this.sidebar_right = L.control.sidebar({
            autopan: false,
            closeButton: true,
            container: 'sideBar-right',
            position: 'right',
        }).addTo(map);
    }

    /* UAV List */
    getUAVList() {

    }

    setUAVList(uav_list) {

    }

    addUAVListCallback(callback) {

    }

    /* UAV */
    getUAV(id) {

    }

    setUAV(id) {

    }

    addUAVCallback(callback) {

    }

    /* Mission List */
    getMissionList() {

    }

    setMissionList(mission_list) {

    }

    addMissionListCallback(callback) {

    }

    /* Confirmed Mission */
    getMission(id) {

    }

    setMission(id) {

    }

    addMissionCallback(callback) {

    }

    /* Draw Mission */
    getDrawMission() {

    }

    setDrawMission() {

    }

    addDrawMissionCallback(callback) {

    }
}





/*

<div id="sideBar">
    <div id="sideBar-left" class="leaflet-sidebar collapsed">

        <!-- nav tabs -->
        <div class="leaflet-sidebar-tabs">
            <!-- top aligned tabs -->
            <ul role="tablist">
                <li><a href="#sideBar-left-home"              role="tab"><i class="fa fa-home"></i></a></li>
                <li><a href="#sideBar-left-missionPlanner"    role="tab"><i class="fas fa-map-marked"></i></a></li>
                <li><a href="#sideBar-left-missionController" role="tab"><i class="fa fa-flag-checkered"></i></a></li>
                <li><a href="#sideBar-left-imageOverlay"      role="tab"><i class="fas fa-gamepad"></i></a></li>
                <li><a href="#sideBar-left-settings"          role="tab"><i class="fas fa-image"></i></a></li>
            </ul>
        </div>

        <div class="leaflet-sidebar-content">
            {% include 'include/SideBar/LeftSideBar/Home/Home.html' %}
            {% include 'include/SideBar/LeftSideBar/MissionPlanner/MissionPlanner.html' %}
            {% include 'include/SideBar/LeftSideBar/MissionController/MissionController.html' %}
            {% include 'include/SideBar/LeftSideBar/UAVController/UAVController.html' %}
            {% include 'include/SideBar/LeftSideBar/ImageOverlay/ImageOverlay.html' %}
            {% include 'include/SideBar/LeftSideBar/Settings/Settings.html' %}
        </div>
    </div>

    <div id="sideBar-right" class="leaflet-sidebar collapsed">

        <!-- nav tabs -->
        <div class="leaflet-sidebar-tabs">
            <!-- top aligned tabs -->
            <ul role="tablist">
                <li><a href="#sideBar-right-drawInfo"    role="tab"><i class="fas fa-map-marked"></i></a></li>
                <li><a href="#sideBar-right-missionInfo" role="tab"><i class="fa fa-flag-checkered"></i></a></li>
                <li><a href="#sideBar-right-UAVInfo"     role="tab"><i class="fas fa-helicopter"></i></a></li>
            </ul>
        </div>

        <div class="leaflet-sidebar-content">
            {% include 'include/SideBar/RightSideBar/DrawInfo/DrawInfo.html' %}
            {% include 'include/SideBar/RightSideBar/MissionInfo/MissionInfo.html' %}
            {% include 'include/SideBar/RightSideBar/UAVInfo/UAVInfo.html' %}
        </div>
    </div>
</div>

*/