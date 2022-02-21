class MissionDrawer
{
    constructor() {
        M.MISSION_MANAGER.addInfoParamCallback('status',  this.updateMissionParam.bind(this));
        M.MISSION_MANAGER.addInfoParamCallback('layers',  this.updateMissionParam.bind(this));

        this.missionListId = Object.assign([], M.MISSION_MANAGER.getInfoList());
        this.missionDict   = Object.assign({}, M.MISSION_MANAGER.getInfoDict());

        this.MISSION_LIST = new SmartList();
    }

    _checkLayer(id, name) {
        if (name in this.MISSION_LIST.getDictById(id)) {
            this.MISSION_LIST.getDictById(id)[name].codeLayerDrawn.remove();
        }
    }

    updateMissionParam(param, value, args) {
        let id = args[0];
        
        console.log("UPDATE MISSION PARAM");
        console.log(id);
        console.log(param);
        console.log(value);
        /*
        if ((this.MISSION_LIST.getList().indexOf(id) === -1)) {
            this.MISSION_LIST.addObject(id, {'id': id});
        }

        if (param in this.MISSION_LIST.getDictById(id)) {
            switch (param) {
                case 'pose':
                    this.MISSION_LIST.getDictById(id)['layerPose'].codeLayerDrawn.setLatLng([value['lat'], value['lng']]);
                    break;

                default:
                    break;
            }

        } else {
            switch (param) {
                case 'pose':
                    this._checkLayer(id, 'layerPose');
                    this.MISSION_LIST.getDictById(id)['layerPose'] = new MissionMarker();
                    this.MISSION_LIST.getDictById(id)['layerPose'].codeDraw(id, [value['lat'], value['lng']]);
                    break;

                default:
                    break;
            }
        }
        this.MISSION_LIST.getDictById(id)[param] = value;   
        */
    }
}