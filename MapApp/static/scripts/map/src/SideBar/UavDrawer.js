class UavDrawer
{
    constructor() {
        M.UAV_MANAGER.addInfoParamCallback('pose', this.updateUavParam.bind(this));
        M.UAV_MANAGER.addInfoParamCallback('odom', this.updateUavParam.bind(this));
        M.UAV_MANAGER.addInfoParamCallback('desiredPath', this.updateUavParam.bind(this));
        M.UAV_MANAGER.addInfoParamCallback('state', this.updateUavParam.bind(this));

        this.uavListId = Object.assign([], M.UAV_MANAGER.getList());
        this.uavDict = Object.assign({}, M.UAV_MANAGER.getDict());

        this.UAV_LIST = new SmartList();

        M.addMapCallback('pm:drawstart', this.onDrawCallback.bind(this), false);
        M.addMapCallback('pm:drawend', this.onDrawCallback.bind(this), true);

        // M.MAP.on('pm:drawstart', (e) => {
        //     console.log("drawstart");
        //     for (let i = 0; i < this.UAV_LIST.getList().length; i++) {
        //         this.UAV_LIST.getDictById(this.UAV_LIST.getList()[i])['layerPose']['popupState'] = false;
        //         updatePopup(id);
        //     }
        // });
        
        // M.MAP.on('pm:drawend', (e) => {
        //     console.log("drawend");
        //     for (let i = 0; i < this.UAV_LIST.getList().length; i++) {
        //         this.UAV_LIST.getDictById(this.UAV_LIST.getList()[i])['layerPose']['popupState'] = true;
        //         this.UAV_LIST.getDictById(this.UAV_LIST.getList()[i])['layerPose']['marker'].openPopup();
        //     }
        // });
        
    }

    _checkLayer(id, name) {
        if (name in this.UAV_LIST.getDictById(id)) {
            this.UAV_LIST.getDictById(id)[name].codeLayerDrawn.remove();
        }
    }

    updateUavParam(param, value, args) {

        let id = args[0];

        if (this.UAV_LIST.getList().indexOf(id) === -1) {
            this.UAV_LIST.addObject(id, {'id': id});
        }

        if (param == 'odom' && value.length < 2 ||
            param == 'desiredPath' && value.length < 2) {
            return;
        }

        if (param in this.UAV_LIST.getDictById(id)) {
            switch (param) {
                case 'pose':
                    this.UAV_LIST.getDictById(id)['layerPose'].codeLayerDrawn.setLatLng([value['lat'], value['lng']]);
                    this.UAV_LIST.getDictById(id)['layerPose'].codeLayerDrawn.options.rotationAngle = this._angleWrap(value['yaw']);
                    this.updatePopup(id);
                    break;
                case 'odom':
                    this.UAV_LIST.getDictById(id)['layerOdom'].codeLayerDrawn.setLatLngs(value);
                    break;
                case 'desiredPath':
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
                    this.UAV_LIST.getDictById(id)['layerPose'].codeDraw(id, [value['lat'], value['lng']], {'rotationAngle': this._angleWrap(value['yaw'])});
                    this.updatePopup(id);
                    break;
                case 'odom':
                    this._checkLayer(id, 'layerOdom'); 
                    this.UAV_LIST.getDictById(id)['layerOdom'] = new Odom();
                    let odomValue = value;
                    if (value.length < 2) {
                        odomValue = [value, value];
                    }
                    this.UAV_LIST.getDictById(id)['layerOdom'].codeDraw(id, odomValue);
                    break;
                case 'desiredPath':
                    this._checkLayer(id, 'layerDesiredPath');
                    this.UAV_LIST.getDictById(id)['layerDesiredPath'] = new DesiredPath();
                    this.UAV_LIST.getDictById(id)['layerDesiredPath'].codeDraw(id, value);
                    break;
                case 'state':
                    this._checkLayer(id, 'layerPose');
                    let pose = M.UAV_MANAGER.getDictById(id)['pose'];
                    this.UAV_LIST.getDictById(id)['pose'] = pose;

                    this.UAV_LIST.getDictById(id)['layerPose'] = new UAVMarker();
                    this.UAV_LIST.getDictById(id)['layerPose'].codeDraw(id, [pose['lat'], pose['lng']]);
                    this.UAV_LIST.getDictById(id)['layerPose'].codeLayerDrawn.options['state'] = value;
                default:
                    break;
            }
        }
        this.UAV_LIST.getDictById(id)[param] = value;
    }

    updatePopup(id) {
        var uavLayer = this.UAV_LIST.getDictById(id)['layerPose'];
        
        let height = M.UAV_MANAGER.getDictById(id)['pose']['height'];
        console.log("Update popup: " + height);
        let popupContent = `<p>Height = ${Utils.round(height, 2)} m</p>`;

        if (uavLayer['popup'] == undefined) {
            uavLayer['marker'] = uavLayer.codeLayerDrawn;

            let popup = L.popup({
                closeOnClick: false,
                autoClose: false
            }).setContent(popupContent);

            uavLayer['marker'].bindPopup(popup).openPopup();
            uavLayer['popup'] = uavLayer['marker'].getPopup();
            uavLayer['popupState'] = true;

            var callback = this.popupListenerCallback.bind(this);

            uavLayer['marker'].on('popupopen', function(e) {
                // console.log('popupopen');
                // console.log(id);
                callback(id, true);
            });

            uavLayer['marker'].on('popupclose', function(e) {
                // console.log('popupclose');
                // console.log(id);
                callback(id, false);
            });

        } else {
            uavLayer['popup'].setContent(popupContent).update();

            if (uavLayer['popupState']) {
                uavLayer['marker'].openPopup(); // TODO: fix not open in onDrawCallback
            } else {
                uavLayer['marker'].closePopup();
            }     
        }
    }

    popupListenerCallback(id, state) {
        this.UAV_LIST.getDictById(id)['layerPose']['popupState'] = state;
    }

    onDrawCallback(args, e) {
        for (let i = 0; i < this.UAV_LIST.getList().length; i++) {
            let id = this.UAV_LIST.getList()[i];
            this.UAV_LIST.getDictById(id)['layerPose']['popupState'] = args[0];
            this.updatePopup(id);
        }
    }

    _angleWrap(angle) {
        angle = angle * 180 / Math.PI ;

        // reduce the angle  
        angle =  angle % 360; 

        // force it to be the positive remainder, so that 0 <= angle < 360  
        angle = - (angle + 360) % 360;  
        
        return angle;
    }
}