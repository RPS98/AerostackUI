
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

    getUavList() {
        return this.uavListId;
    }

    getUavDict() {
        return this.uavDict;
    }

    getUavListById(id) {
        return this.uavDict[id];
    }

    modifyUav(id, state, pose, odom, desiredPath, sensors) {
        this.uavDict.newState(id, state, pose, odom, desiredPath, sensors);
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

    getMissionList() {
        return this.missionListId;
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



var MAP = null;
var MISSION_LIST = new MissionList();
var UAV_LIST = new UAVList();

UAV_LIST.addUav('Virtual UAV', 'landed', {'lat': 0, 'lng': 0}, {'x': 0, 'y': 0}, [], []);
MISSION_LIST.addMission('Virtual Mission', 'confirmed', [], []);


var MAP_MANAGER = null;
var WEB_SOCKET_MANAGER = null;
var DRAW_MANAGER = null;
