class UAV {
    constructor(id, state, pose, odom, desiredPath, sensors) 
    {
        this.newState(id, state, pose, odom, desiredPath, sensors);
    }

    newState(id, state, pose, odom, desiredPath, sensors) 
    {
        this.id = id;
        this.state = state;
        this.pose = pose;
        this.odom = odom;
        this.desiredPath = desiredPath;
        this.sensors = sensors;
    }
}

class Mission
{
    constructor(id, state, uavList, layers) 
    {
        this.newState(id, state, uavList, layers);
    }

    newState(id, state, uavList, layers) 
    {
        this.id = id;
        this.state = state;
        this.uavList = uavList;
        this.layers = layers;
    }
}

class UAVList
{
    constructor() {
        this.uavListId = [];
        this.uavDict = {};
    }

    addUav(id, state, pose, odom, desiredPath, sensors) {
        this.uavListId.push(id);
        this.uavDict[id] = new UAV(id, state, pose, odom, desiredPath, sensors);
    }

    removeUav(id) {
        this.uavListId.splice(this.uavListId.indexOf(id), 1);
        delete this.uavDict[id];
    }

    getUavDict(id) {
        return this.uavDict[id];
    }

    getAllUavList() {
        return this.uavListId;
    }

    getAllUavDict() {
        return this.uavDict;
    }

    setUav(id, state, pose, odom, desiredPath, sensors) {
        this.uavDict.newState(id, state, pose, odom, desiredPath, sensors);
    }

    setUavState(id, state) {
        this.uavDict[id].state = state;
    }

    setUavPose(id, pose) {
        this.uavDict[id].pose = pose;
    }

    setUavOdom(id, odom) {
        this.uavDict[id].odom = odom;
    }

    setUavDesiredPath(id, desiredPath) {
        this.uavDict[id].desiredPath = desiredPath;
    }

    setUavSensors(id, sensors) {
        this.uavDict[id].sensors = sensors;
    }
}


class MissionList
{
    constructor() {
        this.missionListId = [];
        this.missionDict = {};
    }

    addMission(id, status, uavList, layers) {
        this.missionListId.push(id);
        this.missionDict[id] = {'id': id, 'status': status, 'uavList': uavList, 'layers': layers};
    }

    removeMission(id) {
        this.missionListId.splice(this.missionListId.indexOf(id), 1);
        delete this.missionDict[id];
    }

    getMissionDict(id) {
        return this.missionDict[id];
    }

    getAllMissionList() {
        return this.missionListId;
    }

    getAllMissionDict() {
        return this.missionDict;
    }

    getMissionListById(id) {
        return this.missionDict[id];
    }  

    modifyMission(id, status, uavList, layers) {
        this.missionDict.newState(id, status, uavList, layers);
    }

    getLength() {
        return this.missionListId.length;
    }
}


class Layer
{
    constructor(author, missionId, uavList, height='none') {
        this.info = {
            'author': author,
            'missionId': missionId,
            'uavList': uavList,
            'height': height
        }
    }

    getInfo() {
        return this.info;
    }

    updateLayer(author, missionId, uavList, height='none') {
        this.info = {
            'author': author,
            'missionId': missionId,
            'uavList': uavList,
            'height': height
        }
    }

    updateAuthor(author) {
        this.info.author = author;
    }

    updateMissionId(missionId) {
        this.info.missionId = missionId;
    }
    
    updateUavList(uavList) {
        this.info.uavList = uavList;
    }

    updateHeight(height) {
        this.info.height = height;
    }
}


/*
var MAP = null;
var MISSION_LIST = new MissionList();
var UAV_LIST = new UAVList();

UAV_LIST.addUav('Virtual UAV', 'landed', {'lat': 0, 'lng': 0}, {'x': 0, 'y': 0}, [], []);
MISSION_LIST.addMission('Virtual Mission', 'confirmed', [], []);
*/

var M = null;
var WEB_SOCKET_MANAGER = null;
var DRAW_MANAGER = null;
