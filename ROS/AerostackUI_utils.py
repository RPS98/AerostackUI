class UAV():
    def __init__(self, id, state, pose, odom=[], desiredPath=[], sensors={}):
        self.setUav(id, state, pose, odom, desiredPath, sensors)
        
    def getInfo(self):
        return {
            'id': self.id,
            'state': self.state,
            'pose': self.pose,
            'odom': self.odom,
            'desiredPath': self.desiredPath,
            'sensors': self.sensors
        }
        
    def setUav(self, id, state, pose, odom=[], desiredPath=[], sensors={}):
        self.id = id
        self.state = state
        self.pose = pose
        self.odom = odom
        self.desiredPath = desiredPath
        self.sensors = sensors

    def setUavState(self, state):
        self.state = state
        
    def setUavPose(self, pose):
        self.pose = pose

    def setUavOdom(self, odom):
        self.odom = odom

    def setUavDesiredPath(self, desiredPath):
        self.desiredPath = desiredPath

    def setUavSensors(self, sensors):
        self.sensors = sensors
        

class Mission():
    def __init__(self, id, state, uavList, layers):
        self.setMission(id, state, uavList, layers)
        
    def getInfo(self):
        return {
            'id': self.id,
            'state': self.state,
            'uavList': self.uavList,
            'layers': self.layers
        }
        
    def setMission(self, id, state, uavList, layers):
        self.id = id
        self.state = state
        self.uavList = uavList
        self.layers = layers
        
    def setMissionState(self, state):
        self.state = state
        
    def setMissionUavList(self, uavList):
        self.uavList = uavList
        
    def setMissionLayers(self, layers):
        self.layers = layers
        

class SmartList():
    def __init__(self):
        self.objectList = []
        self.objectDict = {}
        
    def getList(self):
        return self.objectList;

    def getDict(self):
        return self.objectDict;
    
    def getDictById(self, id):
        return self.objectDict[id]

    def getDictInfo(self):
        objectDict = []
        for id in self.objectList:
            objectDict.append(self.objectDict[id].getInfo())
        return objectDict;

    def getDictInfoById(self, id):
        return {
            id: self.objectDict[id].getInfo(),
        }

    def removeObject(self, id):
        self.objectList.remove(id);
        del self.objectDict[id];

    def addObject(self, id, object):
        self.objectList.append(id);
        self.objectDict[id] = object;