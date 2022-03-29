class MissionDrawer {
    constructor() {
        M.MISSION_MANAGER.addMissionConfirmCallback(this.missionConfirmCallback.bind(this));
        M.MISSION_MANAGER.addInfoAddCallback(this.newMissionCallback.bind(this));
        // M.MISSION_MANAGER.addInfoParamCallback('status',  this.updateMissionParam.bind(this));
        // M.MISSION_MANAGER.addInfoParamCallback('layers',  this.updateMissionParam.bind(this));

        this.missionListId = Object.assign([], M.MISSION_MANAGER.getList());
        this.missionDict = Object.assign({}, M.MISSION_MANAGER.getDict());

        this.MISSION_LIST = new SmartList();

        this.addDrawTypes();
    }

    addDrawTypes() {
        let status = 'confirmed';
        this.path = new Path(status, undefined, { 'opacity': 0.6, 'weight': 4 });
        this.landPoint = new LandPoint(status);
        this.takeOffPoint = new TakeOffPoint(status);
        this.wayPoint = new WayPoint(status);
        this.area = new Area(status);
    }

    missionConfirmCallback(myargs, args) {
        console.log("missionConfirmCallback");
        console.log(args);
        let missionId = args['oldId'];
        let layers = M.getLayers();

        for (let i = 0; i < layers.length; i++) {
            if (layers[i].pm.options.status == 'draw' &&
                layers[i].pm.options.missionId == missionId) {
                layers[i].remove();
            }
        }
    }

    newMissionCallback(myargs, args) {
        let missionId = args[0];
        let missionDict = M.MISSION_MANAGER.getDictById(missionId);

        console.log("newMissionCallback");
        console.log(missionId);
        console.log(missionDict);

        for (let i = 0; i < missionDict.layers.length; i++) {
            let layer = missionDict.layers[i];
            let uavId = layer['uavList'][0];

            switch (layer.name) {
                case 'TakeOffPoint':
                    this.takeOffPoint.codeDraw([layer.values['lat'], layer.values['lng']], undefined, undefined, uavId);
                    break;
                case 'Path':
                    this.path.codeDraw(layer.values, undefined, undefined, uavId);
                    break;
                case 'LandPoint':
                    this.landPoint.codeDraw([layer.values.lat, layer.values.lng], undefined, undefined, uavId);
                    break;
                case 'WayPoint':
                    this.wayPoint.codeDraw([layer.values.lat, layer.values.lng], undefined, undefined, uavId);
                    break;
                case 'Area':
                    this.area.codeDraw(layer.values[0], undefined, {'opacity': 0.3}, missionId);

                    for (let j = 0; j < layer.uavList.length; j++) {
                        let uavId_aux = layer.uavList[j];
                        this.path.codeDraw(layer.uavPath[uavId_aux], undefined, undefined, uavId);
                    }

                    break;
                default:
                    throw new Error("Unknown layer name: " + layer.name);
                    break;
            }
        }
    }
}