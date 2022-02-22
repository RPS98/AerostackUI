class UavDrawer
{
    constructor() {
        M.UAV_MANAGER.addInfoParamCallback('pose', this.updateUavParam.bind(this));
        M.UAV_MANAGER.addInfoParamCallback('odom', this.updateUavParam.bind(this));
        M.UAV_MANAGER.addInfoParamCallback('desiredPath', this.updateUavParam.bind(this));
        M.UAV_MANAGER.addInfoParamCallback('state', this.updateUavParam.bind(this));

        this.uavListId = Object.assign([], M.UAV_MANAGER.getInfoList());
        this.uavDict = Object.assign({}, M.UAV_MANAGER.getInfoDict());

        this.UAV_LIST = new SmartList();
    }

    _checkLayer(id, name) {
        if (name in this.UAV_LIST.getDictById(id)) {
            this.UAV_LIST.getDictById(id)[name].codeLayerDrawn.remove();
        }
    }

    updateUavParam(param, value, args) {
        let id = args[0];

        console.log("Uav update param");

        if ((this.UAV_LIST.getList().indexOf(id) === -1)) {
            this.UAV_LIST.addObject(id, {'id': id});
        }

        if (param in this.UAV_LIST.getDictById(id)) {
            switch (param) {
                case 'pose':
                    this.UAV_LIST.getDictById(id)['layerPose'].codeLayerDrawn.setLatLng([value['lat'], value['lng']]);
                    break;
                case 'odom':
                    this.UAV_LIST.getDictById(id)['layerOdom'].codeLayerDrawn.setLatLngs(value);
                    break;
                case 'desiredPath':
                    console.log("desiredPath2");
                    console.log(value);
                    this.UAV_LIST.getDictById(id)['layerDesiredPath'].codeLayerDrawn.setLatLngs(value);
                    break;
                case 'state':
                    this.UAV_LIST.getDictById(id)['layerPose'].codeLayerDrawn.options['state'] = value;
                    break;
                default:
                    break;
            }

        } else {
            switch (param) {
                case 'pose':
                    this._checkLayer(id, 'layerPose');
                    this.UAV_LIST.getDictById(id)['layerPose'] = new UAVMarker();
                    this.UAV_LIST.getDictById(id)['layerPose'].codeDraw(id, [value['lat'], value['lng']]);
                    break;
                case 'odom':
                    this._checkLayer(id, 'layerOdom'); 
                    this.UAV_LIST.getDictById(id)['layerOdom'] = new Odom();
                    this.UAV_LIST.getDictById(id)['layerOdom'].codeDraw(id, value);
                    break;
                case 'desiredPath':
                    console.log("desiredPath");
                    console.log(value);
                    this._checkLayer(id, 'layerDesiredPath');
                    this.UAV_LIST.getDictById(id)['layerDesiredPath'] = new DesiredPath();
                    this.UAV_LIST.getDictById(id)['layerDesiredPath'].codeDraw(id, value);
                    break;
                case 'state':
                    this._checkLayer(id, 'layerPose');
                    let pose = M.UAV_MANAGER.getInfoDictById(id)['pose'];

                    this.UAV_LIST.getDictById(id)['layerPose'] = new UAVMarker();
                    this.UAV_LIST.getDictById(id)['layerPose'].codeDraw(id, [pose['lat'], pose['lng']]);
                    this.UAV_LIST.getDictById(id)['layerPose'].codeLayerDrawn.options['state'] = value;
                default:
                    break;
            }
        }
        this.UAV_LIST.getDictById(id)[param] = value;
    }
}