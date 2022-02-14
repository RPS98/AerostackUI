class UAV():
    def __init__(self, id, state, pose, odom=[], desired_path=[], sensors={}):
        self.set_uav(id, state, pose, odom, desired_path, sensors)
        
    def get_info(self):
        return {
            'id': self.id,
            'state': self.state,
            'pose': self.pose,
            'odom': self.odom,
            'desiredPath': self.desired_path,
            'sensors': self.sensors
        }
        
    def set_uav(self, id, state, pose, odom=[], desired_path=[], sensors={}):
        self.id = id
        self.state = state
        self.pose = pose
        self.odom = odom
        self.desired_path = desired_path
        self.sensors = sensors

    def set_uav_state(self, state):
        self.state = state
        
    def set_uav_pose(self, pose):
        self.pose = pose

    def set_uav_odom(self, odom):
        self.odom = odom

    def set_uav_desired_path(self, desired_path):
        self.desired_path = desired_path

    def set_uav_sensors(self, sensors):
        self.sensors = sensors
        

class SmartList():
    def __init__(self):
        self.object_list = []
        self.object_dict = {}
        
    def get_list(self):
        return self.object_list;

    def get_dict(self):
        return self.object_dict;
    
    def get_dict_by_id(self, id):
        return self.object_dict[id]

    def get_dict_info(self):
        object_dict = []
        for id in self.object_list:
            object_dict.append(self.object_dict[id].get_info())
        return object_dict;

    def get_dict_info_by_id(self, id):
        return {
            id: self.object_dict[id].get_info(),
        }

    def remove_object(self, id):
        self.object_list.remove(id);
        del self.object_dict[id];

    def add_object(self, id, object):
        self.object_list.append(id);
        self.object_dict[id] = object;
        
        
class UAV_MANAGER():
    def __init__(self, sender):
        self.UAV_LIST = SmartList()
        self.sender = sender
    
    def add_uav(self, id, state, pose, odom=[], desired_path=[], sensors={}):
        self.UAV_LIST.addObject(id, UAV(id, state, pose, odom, desired_path, sensors))
        
    def remove_uav(self, id):
        self.remove_uav(id)
        
    def update_uav(self, info_dictionary, overwrite=False):
        id = info_dictionary['id']
        
        if id in self.UAV_LIST.get_list():
            for key in info_dictionary:
                uav = self.UAV_LIST.get_dict_by_id(id)
                uav[key] = info_dictionary[key]
        
        else:
            if (info_dictionary['id'] and
                info_dictionary['state'] and
                info_dictionary['pose']):
                
                self.add_uav(info_dictionary['id'], info_dictionary)
                
            else:
                raise Exception('Invalid UAV info')
        
        
        