class MissionDrawer
{
    constructor() {
        M.MISSION_MANAGER.addMissionConfirmCallback(this.missionConfirmCallback.bind(this));
        M.MISSION_MANAGER.addInfoAddCallback(this.newMissionCallback.bind(this));
        // M.MISSION_MANAGER.addInfoParamCallback('status',  this.updateMissionParam.bind(this));
        // M.MISSION_MANAGER.addInfoParamCallback('layers',  this.updateMissionParam.bind(this));

        this.missionListId = Object.assign([], M.MISSION_MANAGER.getInfoList());
        this.missionDict   = Object.assign({}, M.MISSION_MANAGER.getInfoDict());

        this.MISSION_LIST = new SmartList();

        this.addDrawTypes();
    }

    addDrawTypes() {
        this.path = new Path();
        this.landPoint = new LandPoint();
        this.takeOffPoint = new TakeOffPoint();
    }

    missionConfirmCallback(myargs, args) {
        console.log("missionConfirmCallback");
        console.log(args);
        let missionId = args['oldId'];
        let layers = M.getLayers();

        for (let i=0; i<layers.length; i++) {
            if (layers[i].pm.options.status == 'draw' &&
                layers[i].pm.options.missionId == missionId) {
                    layers[i].remove();
            }
        }   
    }

    newMissionCallback(myargs, args) {
        
        let missionDict = M.MISSION_MANAGER.getInfoDictById(args);
        let uavId = missionDict['uavList'][0];

        for (let i=0; i<missionDict.layers.length; i++) {
            let layer = missionDict.layers[i];

            
            switch (layer.name) {
                case 'TakeOffPoint':
                    this.takeOffPoint.codeDraw(uavId, [layer.values['lat'], layer.values['lng']]);
                    break;
                case 'Path':
                    this.path.codeDraw(layer.values, {color: M.UAV_MANAGER.getColors(uavId)[1]});
                    break;
                case 'LandPoint':
                    this.landPoint.codeDraw(uavId, [layer.values.lat, layer.values.lng]);
                    break;
                default:
                    throw new Error("Unknown layer name: " + layer.name);
                    break;
            }
        }
        
    }

    /*
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
    }
    */
}