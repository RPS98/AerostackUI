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

        this.status = 'uav';

        this.uavMarker = new UAVMarker(this.status);
        this.odom = new Odom(this.status);
        this.desiredPath = new DesiredPath(this.status, undefined, {'opacity': 0.5});        
    }

    _checkLayer(id, name) {
        if (name in this.UAV_LIST.getDictById(id)) {
            this.UAV_LIST.getDictById(id)[name].codeLayerDrawn.remove();
        }
    }

    updateUavParam(param, value, args) {
        let uavId = args[0];

        if (this.UAV_LIST.getList().indexOf(uavId) === -1) {
            this.UAV_LIST.addObject(uavId, {'id': uavId});
        }

        if (param == 'odom' && value.length < 2 ||
            param == 'desiredPath' && value.length < 2) {
            return;
        }

        

        if (param in this.UAV_LIST.getDictById(uavId)) {
            switch (param) {
                case 'pose':
                    this.UAV_LIST.getDictById(uavId)['layerPose'].codeLayerDrawn.setLatLng([value['lat'], value['lng']]);
                    this.UAV_LIST.getDictById(uavId)['layerPose'].codeLayerDrawn.options.rotationAngle = this._angleWrap(value['yaw']);
                    this.updatePopup(uavId);
                    break;
                case 'odom':
                    this.UAV_LIST.getDictById(uavId)['layerOdom'].codeLayerDrawn.setLatLngs(value);
                    break;
                case 'desiredPath':
                    this.UAV_LIST.getDictById(uavId)['layerDesiredPath'].codeLayerDrawn.setLatLngs(value);
                    break;
                case 'state':
                    this.UAV_LIST.getDictById(uavId)['layerPose'].codeLayerDrawn.options['state'] = value;
                    break;
                default:
                    break;
            }

        } else {
            switch (param) {
                case 'pose':
                    this._checkLayer(uavId, 'layerPose');
                    let colors = M.UAV_MANAGER.getColors(uavId);
                    this.UAV_LIST.getDictById(uavId)['layerPose'] = this.uavMarker;
                    this.UAV_LIST.getDictById(uavId)['layerPose'].codeDraw([value['lat'], value['lng']], {'uavList': [uavId]}, {'rotationAngle': this._angleWrap(value['yaw'])}, uavId);
                    this.updatePopup(uavId);
                    ConsoleSideBar.addMessage(`UAV ${uavId} added`);
                    break;
                case 'odom':
                    this._checkLayer(uavId, 'layerOdom'); 
                    this.UAV_LIST.getDictById(uavId)['layerOdom'] = this.odom;
                    let odomValue = value;
                    if (value.length < 2) {
                        odomValue = [value, value];
                    }
                    this.UAV_LIST.getDictById(uavId)['layerOdom'].codeDraw(odomValue, {'uavList': [uavId]}, undefined, uavId);
                    break;
                case 'desiredPath':
                    this._checkLayer(uavId, 'layerDesiredPath');
                    this.UAV_LIST.getDictById(uavId)['layerDesiredPath'] = this.desiredPath
                    this.UAV_LIST.getDictById(uavId)['layerDesiredPath'].codeDraw(value, {'uavList': [uavId]}, undefined, uavId);
                    break;
                case 'state':
                    this._checkLayer(uavId, 'layerPose');
                    let pose = M.UAV_MANAGER.getDictById(uavId)['pose'];
                    this.UAV_LIST.getDictById(uavId)['pose'] = pose;

                    this.UAV_LIST.getDictById(uavId)['layerPose'] = this.uavMarker;
                    this.UAV_LIST.getDictById(uavId)['layerPose'].codeDraw([pose['lat'], pose['lng']], {'uavList': [uavId]}, undefined, uavId);
                    this.UAV_LIST.getDictById(uavId)['layerPose'].options.drawManager.options['state'] = value;
                default:
                    break;
            }
        }
        this.UAV_LIST.getDictById(uavId)[param] = value;
    }

    updatePopup(id) {
        var uavLayer = this.UAV_LIST.getDictById(id)['layerPose'];
        
        let height = M.UAV_MANAGER.getDictById(id)['pose']['height'];
        // console.log("Update popup: " + height);
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